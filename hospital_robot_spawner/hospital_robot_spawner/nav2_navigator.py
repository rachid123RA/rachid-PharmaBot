#!/usr/bin/env python3
"""
nav2_navigator.py — Navigation Nav2 pour PharmaBot
PharmaBot | Ibn Tofaïl University | 2025-2026
Prof. Khaoula Boukir — Systèmes Embarqués Temps Réel

Remplace le contrôleur proportionnel de navigation_bridge.py par une
navigation Nav2 réelle (action NavigateToPose).

Ce nœud :
    1. Écoute /pharmabot/tache_courante (décision EDF scheduler)
    2. Traduit le département en coordonnées GPS
    3. Envoie un goal Nav2 (action NavigateToPose)
    4. Surveille la progression et publie le statut
    5. Confirme la livraison via /pharmabot/livraison_confirmee
    6. Gère les interruptions HARD_RT (cancel + nouveau goal)
    7. Retour automatique à la pharmacie après livraison
"""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

RESET  = "\033[0m"
BOLD   = "\033[1m"
RED    = "\033[91m"
GREEN  = "\033[92m"
ORANGE = "\033[93m"
CYAN   = "\033[96m"

# Coordonnées des salles — identiques à delivery_detector.py
SALLES = {
    "pharmacie":    {"nom": "Pharmacie",    "x":  1.0,  "y":  16.0, "yaw": 0.0},
    "reanimation":  {"nom": "Réanimation",  "x":  9.5,  "y":   5.5, "yaw": 1.5708},
    "urgences":     {"nom": "Urgences",     "x": -3.5,  "y":  -6.6, "yaw": 1.5708},
    "consultation": {"nom": "Consultation", "x": -0.5,  "y": -25.5, "yaw": 1.5708},
}

DELAI_CHARGEMENT = 3.0   # secondes de "prise" du médicament en pharmacie
TIMEOUT_NAVIGATION = 120  # secondes max pour atteindre une cible


class Nav2Navigator(Node):
    """Navigation Nav2 PharmaBot — gestion complète des missions de livraison."""

    def __init__(self):
        super().__init__("nav2_navigator")
        self.get_logger().info(f"{BOLD}{CYAN}Nav2 Navigator PharmaBot démarré{RESET}")

        # ── État interne ──
        self._mission_courante   = None
        self._dept_cible         = None
        self._phase              = "IDLE"
        self._pos_x              = 1.0
        self._pos_y              = 16.0
        self._goal_handle        = None
        self._goal_future        = None
        self._mission_interrompue = None
        self._temps_debut_mission = None
        self._nb_livraisons      = 0
        self._chargement_debut   = None

        # ── Action client Nav2 ──
        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        # ── Publishers ──
        self._pub_nav_status  = self.create_publisher(String, "/pharmabot/nav_status", 10)
        self._pub_livraison   = self.create_publisher(String, "/pharmabot/livraison_confirmee", 10)
        self._pub_nav_goal    = self.create_publisher(String, "/pharmabot/navigation_goal", 10)
        self._pub_log         = self.create_publisher(String, "/pharmabot/mission_log", 10)

        # ── Subscribers ──
        self.create_subscription(String, "/pharmabot/tache_courante",
                                 self._cb_nouvelle_mission, 10)
        self.create_subscription(String, "/pharmabot/edf_interruption",
                                 self._cb_interruption, 10)
        self.create_subscription(String, "/pharmabot/watchdog",
                                 self._cb_watchdog, 10)
        self.create_subscription(Odometry, "/demo/odom",
                                 self._cb_odom, 5)

        # ── Timers ──
        self.create_timer(0.5, self._timer_surveillance)
        self.create_timer(1.0, self._publier_status)

        self.get_logger().info("En attente de Nav2 action server...")
        self._nav_client.wait_for_server(timeout_sec=30.0)
        self.get_logger().info(f"{GREEN}Nav2 connecté — prêt{RESET}")

    # ══════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════

    def _cb_odom(self, msg: Odometry):
        self._pos_x = msg.pose.pose.position.x
        self._pos_y = msg.pose.pose.position.y

    def _cb_nouvelle_mission(self, msg: String):
        try:
            mission = json.loads(msg.data)
            dept    = mission.get("departement", "")
            if dept not in SALLES:
                return
            if self._phase == "IDLE":
                self._demarrer_mission(mission)
        except json.JSONDecodeError:
            pass

    def _cb_interruption(self, msg: String):
        """Interruption EDF HARD_RT — cancel goal actuel, mission urgente."""
        try:
            data = json.loads(msg.data)
            dept_urgent = data.get("dept_urgent", "")
            if dept_urgent not in SALLES:
                return
            if self._phase in ("IDLE",):
                return

            self.get_logger().warn(
                f"{RED}{BOLD}[EDF OVERRIDE]{RESET} "
                f"{self._dept_cible} → {dept_urgent}")

            # Sauvegarder mission interrompue
            if self._mission_courante:
                self._mission_interrompue = self._mission_courante

            # Annuler le goal en cours
            self._annuler_navigation()

            # Démarrer mission urgente
            mission_urgente = {
                "departement":  dept_urgent,
                "type_rt":      data.get("type_rt_urgent", "HARD_RT"),
                "deadline_sec": data.get("deadline_urgent", 30),
                "medicament":   data.get("medicament_urgent", "Médicament urgent"),
                "timestamp":    time.time(),
            }
            self._demarrer_mission(mission_urgente)
        except json.JSONDecodeError:
            pass

    def _cb_watchdog(self, msg: String):
        try:
            data = json.loads(msg.data)
            if data.get("type") == "RECUPERATION":
                self._annuler_navigation()
                self._phase = "IDLE"
        except json.JSONDecodeError:
            pass

    # ══════════════════════════════════════════════
    # Gestion missions
    # ══════════════════════════════════════════════

    def _demarrer_mission(self, mission: dict):
        self._mission_courante    = mission
        self._temps_debut_mission = time.time()
        self._phase               = "VERS_PHARMACIE"
        dept = mission.get("departement", "?")
        med  = mission.get("medicament", "?")
        self.get_logger().info(
            f"{CYAN}[MISSION]{RESET} → {dept} | {med}")
        # D'abord aller à la pharmacie chercher le médicament
        self._envoyer_goal("pharmacie", callback_arrive=self._cb_arrive_pharmacie)

    def _cb_arrive_pharmacie(self, success: bool):
        if not success:
            self.get_logger().error("Impossible d'atteindre la pharmacie")
            self._phase = "IDLE"
            return
        dept = self._mission_courante.get("departement", "?")
        med  = self._mission_courante.get("medicament", "?")
        self.get_logger().info(
            f"{GREEN}[PHARMACIE]{RESET} Arrivé — chargement {med}...")
        self._phase             = "CHARGEMENT"
        self._chargement_debut  = time.time()
        # Attendre DELAI_CHARGEMENT secondes puis partir vers destination
        self.create_timer(DELAI_CHARGEMENT, self._partir_vers_destination)

    def _partir_vers_destination(self):
        if not self._mission_courante or self._phase != "CHARGEMENT":
            return
        dept = self._mission_courante.get("departement", "?")
        self._phase = "NAVIGATION"
        self.get_logger().info(
            f"{CYAN}[DÉPART]{RESET} Médicament chargé → {dept}")
        self._pub_log.publish(self._json_msg({
            "event": "depart_livraison",
            "dest":  dept,
            "medicament": self._mission_courante.get("medicament"),
        }))
        self._envoyer_goal(dept, callback_arrive=self._cb_arrive_destination)

    def _cb_arrive_destination(self, success: bool):
        if not self._mission_courante:
            return
        dept     = self._mission_courante.get("departement", "?")
        deadline = self._mission_courante.get("deadline_sec", 999)
        type_rt  = self._mission_courante.get("type_rt", "?")
        elapsed  = time.time() - self._temps_debut_mission
        ok       = elapsed <= deadline
        self._nb_livraisons += 1

        c = GREEN if ok else ORANGE
        self.get_logger().info(
            f"{c}{BOLD}[LIVRAISON #{self._nb_livraisons}]{RESET} "
            f"{dept} | {elapsed:.0f}s / {deadline}s "
            f"| {'✓ OK' if ok else '✗ DÉPASSÉ'}")

        # Publier confirmation
        self._pub_livraison.publish(self._json_msg({
            "departement":     dept,
            "temps_livraison": round(elapsed, 1),
            "deadline_ok":     ok,
            "livraison_num":   self._nb_livraisons,
            "type_rt":         type_rt,
            "medicament":      self._mission_courante.get("medicament"),
            "timestamp":       time.time(),
        }))

        self._phase = "RETOUR"
        # Reprendre mission interrompue ou IDLE
        mission_suivante = self._mission_interrompue
        self._mission_interrompue = None
        self._mission_courante    = None
        self._dept_cible          = None

        if mission_suivante:
            self.get_logger().info(
                f"{CYAN}[EDF REPRISE]{RESET} → {mission_suivante.get('departement')}")
            self._demarrer_mission(mission_suivante)
        else:
            # Retour pharmacie comme base
            self._phase = "RETOUR_BASE"
            self._envoyer_goal("pharmacie", callback_arrive=lambda s: self._set_idle())

    def _set_idle(self):
        self._phase = "IDLE"
        self.get_logger().info(f"{GREEN}[IDLE]{RESET} Retour pharmacie — en attente")

    # ══════════════════════════════════════════════
    # Navigation Nav2
    # ══════════════════════════════════════════════

    def _envoyer_goal(self, dept_key: str, callback_arrive=None):
        """Envoie un NavigateToPose goal vers le département donné."""
        if dept_key not in SALLES:
            return
        salle = SALLES[dept_key]
        self._dept_cible = dept_key

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.pose.position.x = salle["x"]
        pose.pose.position.y = salle["y"]
        pose.pose.position.z = 0.0

        # Orientation quaternion depuis yaw
        yaw = salle["yaw"]
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        goal_msg.pose = pose

        self.get_logger().info(
            f"[NAV2] Goal → {salle['nom']} ({salle['x']:.1f}, {salle['y']:.1f})")

        # Publier navigation_goal pour delivery_detector
        self._pub_nav_goal.publish(self._json_msg({
            "departement": dept_key,
            "x": salle["x"],
            "y": salle["y"],
            "timestamp": time.time(),
        }))

        send_future = self._nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._cb_feedback
        )
        send_future.add_done_callback(
            lambda f: self._cb_goal_response(f, callback_arrive)
        )

    def _cb_goal_response(self, future, callback_arrive):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("[NAV2] Goal refusé par Nav2")
            if callback_arrive:
                callback_arrive(False)
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda f: self._cb_result(f, callback_arrive)
        )

    def _cb_result(self, future, callback_arrive):
        result = future.result()
        status = result.status
        success = (status == GoalStatus.STATUS_SUCCEEDED)
        if not success:
            self.get_logger().warn(
                f"[NAV2] Navigation terminée avec statut : {status}")
        if callback_arrive:
            callback_arrive(success)

    def _cb_feedback(self, feedback_msg):
        """Feedback Nav2 — suivi position en temps réel."""
        fb = feedback_msg.feedback
        dist = fb.distance_remaining if hasattr(fb, 'distance_remaining') else 0
        if dist > 0.5:
            self.get_logger().debug(
                f"[NAV2] Distance restante : {dist:.1f}m")

    def _annuler_navigation(self):
        if self._goal_handle is not None:
            try:
                cancel_future = self._goal_handle.cancel_goal_async()
                self.get_logger().info("[NAV2] Goal annulé (interruption EDF)")
            except Exception:
                pass
            self._goal_handle = None

    # ══════════════════════════════════════════════
    # Surveillance timeout
    # ══════════════════════════════════════════════

    def _timer_surveillance(self):
        if not self._mission_courante or not self._temps_debut_mission:
            return
        elapsed = time.time() - self._temps_debut_mission
        deadline = self._mission_courante.get("deadline_sec", 999)
        type_rt  = self._mission_courante.get("type_rt", "")
        dept     = self._mission_courante.get("departement", "?")

        # Alerte deadline imminente
        restant = deadline - elapsed
        if 0 < restant < 10 and type_rt == "HARD_RT":
            self.get_logger().warn(
                f"{RED}[ALERTE]{RESET} {dept} deadline dans {restant:.0f}s !")

        # Timeout navigation globale
        if elapsed > TIMEOUT_NAVIGATION and self._phase not in ("IDLE", "CHARGEMENT"):
            self.get_logger().error(
                f"[TIMEOUT] Mission {dept} abandonnée après {TIMEOUT_NAVIGATION}s")
            self._annuler_navigation()
            self._mission_courante = None
            self._phase = "IDLE"

    def _publier_status(self):
        self._pub_nav_status.publish(self._json_msg({
            "phase":          self._phase,
            "dept_cible":     self._dept_cible,
            "pos_x":          round(self._pos_x, 2),
            "pos_y":          round(self._pos_y, 2),
            "nb_livraisons":  self._nb_livraisons,
            "mission_active": bool(self._mission_courante),
            "timestamp":      time.time(),
        }))

    def _json_msg(self, data: dict) -> String:
        msg = String()
        msg.data = json.dumps(data)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = Nav2Navigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
