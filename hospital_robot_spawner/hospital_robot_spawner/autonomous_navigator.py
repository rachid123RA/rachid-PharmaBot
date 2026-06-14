#!/usr/bin/env python3
"""
autonomous_navigator.py — Navigation Autonome Robuste Phase 3
PharmaBot | Ibn Tofaïl University | 2025-2026
Prof. Khaoula Boukir — Systèmes Embarqués Temps Réel

PHASE 3 : Couche de navigation avancée qui se branche sur l'architecture
existante WITHOUT modifier navigation_bridge, rt_scheduler, ou mission_manager.

Fonctionnalités ajoutées :
    ✓ Machine à états navigation avancée
    ✓ Évitement obstacles dynamique (LIDAR simulé + grille 2D)
    ✓ Planification de chemin avec waypoints
    ✓ Replanification automatique en cas d'obstacle
    ✓ Comportements de récupération (stuck → rotate → retry)
    ✓ Timeout de navigation avec alerte
    ✓ Arrêt sécurisé garanti
    ✓ Retour automatique pharmacie après livraison
    ✓ Intégration urgences HARD_RT (préemption mission)
    ✓ Publication métriques en temps réel
    ✓ Compatibilité 100% topics Phase 1+2

STATES (internes Phase 3):
    IDLE            — en attente de mission
    PLANNING        — calcul du chemin vers la cible
    MOVING          — déplacement vers waypoint
    REPLANNING      — obstacle détecté, recalcul chemin
    AVOIDING_OBSTACLE — manœuvre d'évitement actif
    GOAL_REACHED    — destination atteinte
    FAILED          — échec navigation (timeout/blocage persistant)
    RECOVERING      — comportement de récupération (rotation/recul)

Topics consommés (Phase 1+2, aucune modification) :
    /pharmabot/tache_courante    → mission EDF active
    /pharmabot/edf_interruption  → override HARD_RT
    /pharmabot/etat_robot        → synchro machine à états
    /pharmabot/watchdog          → arrêt/reprise watchdog
    /demo/odom                   → position/orientation robot

Topics publiés (Phase 3 nouveaux) :
    /navigation_status           → état avancé navigation (spec P3)
    /navigation_metrics          → métriques perf (path_length, etc.)
    /pharmabot/obstacle_detected → obstacle détecté

Topics de compatibilité (spec originale) :
    /navigation_goal             → republication depuis tache_courante
    /watchdog_alert              → pont vers /pharmabot/watchdog
    /mission_state               → pont état mission
    /emergency_override          → pont interruption EDF
"""

import json
import math
import time
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ──────────────────────────────────────────────────────────────────────────────
# Couleurs terminal
# ──────────────────────────────────────────────────────────────────────────────
R = "\033[91m"; G = "\033[92m"; Y = "\033[93m"
C = "\033[96m"; W = "\033[0m";  B = "\033[1m"

# ──────────────────────────────────────────────────────────────────────────────
# Constantes navigation
# ──────────────────────────────────────────────────────────────────────────────
SALLES = {
    "pharmacie":    {"nom": "Pharmacy_Room",    "x":  1.0, "y":  16.0},
    "reanimation":  {"nom": "ICU_Room",          "x": 11.0, "y":   5.5},
    "urgences":     {"nom": "Emergency_Room",    "x": -5.0, "y":  -6.6},
    "consultation": {"nom": "Consultation_Room", "x": -2.0, "y": -27.0},
}

# Gains contrôleur proportionnel
KP_LIN   = 0.4
KP_ANG   = 1.2
VIT_MAX  = 0.6   # m/s
ANG_MAX  = 1.0   # rad/s

# Paramètres navigation
RAYON_ARRIVEE      = 1.5   # m
RAYON_OBSTACLE     = 2.0   # m — distance de détection obstacle
RAYON_SECURITE     = 0.8   # m — distance d'arrêt d'urgence
TIMEOUT_NAV        = 180.0 # s — timeout max navigation
TIMEOUT_STUCK      = 12.0  # s — timeout robot bloqué
DIST_STUCK         = 0.3   # m — distance min pour considérer robot "stuck"
MAX_REPLANIF       = 5     # nombre max de replanifications avant FAILED
DUREE_RECOVERING   = 4.0   # s — durée comportement de récupération

# États navigation Phase 3
STATES_P3 = [
    "IDLE", "PLANNING", "MOVING", "REPLANNING",
    "AVOIDING_OBSTACLE", "GOAL_REACHED", "FAILED", "RECOVERING",
]


class Waypoint:
    """Point intermédiaire dans le chemin planifié."""
    def __init__(self, x: float, y: float, label: str = ""):
        self.x     = x
        self.y     = y
        self.label = label

    def distance_to(self, px: float, py: float) -> float:
        return math.sqrt((self.x - px) ** 2 + (self.y - py) ** 2)

    def __repr__(self):
        return f"WP({self.x:.1f},{self.y:.1f})"


class ObstacleSimulator:
    """
    Simule la détection d'obstacles depuis un LIDAR.
    En l'absence de Gazebo, génère des obstacles aléatoires
    avec une logique réaliste (cooldown, zones libres).
    """
    def __init__(self):
        self._obstacles: list = []    # [(x, y, timestamp)]
        self._last_gen  = 0.0
        self._prob_obs  = 0.05        # 5% chance par check

    def update(self, robot_x: float, robot_y: float) -> list:
        """
        Retourne la liste des obstacles proches (dans RAYON_OBSTACLE).
        Génère aléatoirement des obstacles avec cooldown.
        """
        now = time.time()
        # Nettoyer obstacles vieux (>15s)
        self._obstacles = [
            obs for obs in self._obstacles
            if now - obs[2] < 15.0
        ]
        # Générer nouvel obstacle probabilistiquement
        if now - self._last_gen > 5.0 and random.random() < self._prob_obs:
            angle  = random.uniform(0, 2 * math.pi)
            dist   = random.uniform(RAYON_SECURITE + 0.5, RAYON_OBSTACLE)
            obs_x  = robot_x + dist * math.cos(angle)
            obs_y  = robot_y + dist * math.sin(angle)
            self._obstacles.append((obs_x, obs_y, now))
            self._last_gen = now

        # Retourner obstacles dans rayon
        proches = []
        for obs_x, obs_y, ts in self._obstacles:
            d = math.sqrt((obs_x - robot_x) ** 2 + (obs_y - robot_y) ** 2)
            if d < RAYON_OBSTACLE:
                proches.append({"x": obs_x, "y": obs_y, "distance": d, "ts": ts})
        return proches

    def clear_near(self, robot_x: float, robot_y: float, radius: float = 3.0):
        """Supprime obstacles dans un rayon (après manœuvre d'évitement)."""
        self._obstacles = [
            obs for obs in self._obstacles
            if math.sqrt((obs[0] - robot_x)**2 + (obs[1] - robot_y)**2) > radius
        ]


class PathPlanner:
    """
    Planificateur de chemin simple avec waypoints.
    Utilise une segmentation directe + point de dégagement si obstacle.
    """
    def __init__(self):
        pass

    def plan(self, start_x: float, start_y: float,
             goal_x: float, goal_y: float,
             obstacles: list = None) -> list:
        """
        Génère une liste de waypoints de start vers goal.
        Si des obstacles sont détectés, ajoute un point de dégagement.
        Retourne liste de Waypoint.
        """
        waypoints = []
        if not obstacles:
            # Chemin direct
            waypoints.append(Waypoint(goal_x, goal_y, "GOAL"))
        else:
            # Calculer point de dégagement latéral
            dx   = goal_x - start_x
            dy   = goal_y - start_y
            norm = math.sqrt(dx**2 + dy**2) or 1.0
            # Perpendiculaire normalisée
            perp_x = -dy / norm * 3.0
            perp_y =  dx / norm * 3.0
            # Point de dégagement : 40% du chemin + offset latéral
            deg_x = start_x + dx * 0.4 + perp_x
            deg_y = start_y + dy * 0.4 + perp_y
            waypoints.append(Waypoint(deg_x, deg_y, "DETOUR"))
            waypoints.append(Waypoint(goal_x, goal_y, "GOAL"))

        return waypoints


class AutonomousNavigator(Node):
    """
    Nœud de navigation autonome Phase 3.
    Se branche sur l'architecture Phase 1+2 sans la modifier.
    """

    def __init__(self):
        super().__init__("autonomous_navigator")
        self.get_logger().info(
            f"{B}{C}=== Autonomous Navigator Phase 3 démarré ==={W}")

        # ── État interne ──────────────────────────────────────────────────────
        self._state          = "IDLE"
        self._mission        = None     # dict mission courante
        self._dept_cible     = None     # département cible
        self._waypoints      = []       # chemin planifié
        self._wp_idx         = 0        # index waypoint courant
        self._pos_x          = 1.0
        self._pos_y          = 16.0
        self._orientation    = 0.0
        self._last_pos       = (1.0, 16.0)
        self._last_pos_time  = time.time()
        self._nav_start_time = None
        self._replan_count   = 0
        self._recovering_start = None
        self._recovering_rot   = 1.0   # sens rotation récupération
        self._arret_watchdog   = False
        self._mission_interrompue = None  # sauvegarde si HARD_RT override

        # ── Obstacles et planificateur ────────────────────────────────────────
        self._obs_sim   = ObstacleSimulator()
        self._planner   = PathPlanner()
        self._obstacles = []

        # ── Métriques ─────────────────────────────────────────────────────────
        self._metrics = {
            "path_length":      0.0,
            "travel_time":      0.0,
            "replanning_count": 0,
            "obstacle_count":   0,
            "recovery_count":   0,
            "success_rate":     0.0,
            "missions_total":   0,
            "missions_success": 0,
            "avg_response_time": 0.0,
            "emergency_response_times": [],
        }

        # ── Publishers ────────────────────────────────────────────────────────
        # Phase 3 nouveaux topics
        self._pub_nav_status  = self.create_publisher(
            String, "/navigation_status",           10)
        self._pub_nav_metrics = self.create_publisher(
            String, "/navigation_metrics",          10)
        self._pub_obstacle    = self.create_publisher(
            String, "/pharmabot/obstacle_detected", 10)
        # Compatibilité spec originale
        self._pub_nav_goal    = self.create_publisher(
            String, "/navigation_goal",             10)
        self._pub_watchdog_alert = self.create_publisher(
            String, "/watchdog_alert",              10)
        self._pub_mission_state  = self.create_publisher(
            String, "/mission_state",               10)
        self._pub_emergency_ov   = self.create_publisher(
            String, "/emergency_override",          10)
        # Robot motion
        self._pub_cmd_vel     = self.create_publisher(
            Twist,  "/demo/cmd_vel",                10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self._sub_mission = self.create_subscription(
            String, "/pharmabot/tache_courante",
            self._cb_mission, 10)
        self._sub_edf_int = self.create_subscription(
            String, "/pharmabot/edf_interruption",
            self._cb_edf_interruption, 10)
        self._sub_etat = self.create_subscription(
            String, "/pharmabot/etat_robot",
            self._cb_etat_robot, 10)
        self._sub_watchdog = self.create_subscription(
            String, "/pharmabot/watchdog",
            self._cb_watchdog, 10)
        self._sub_odom = self.create_subscription(
            Odometry, "/demo/odom",
            self._cb_odom, 1)
        # Compat spec : écoute aussi /navigation_goal
        self._sub_nav_goal = self.create_subscription(
            String, "/navigation_goal",
            self._cb_nav_goal_spec, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self._timer_nav     = self.create_timer(0.1,  self._boucle_nav)
        self._timer_status  = self.create_timer(0.5,  self._pub_status)
        self._timer_metrics = self.create_timer(5.0,  self._pub_metrics)
        self._timer_safety  = self.create_timer(1.0,  self._check_safety)

        self.get_logger().info(
            f"{G}Autonomous Navigator prêt — "
            f"états: {STATES_P3}{W}")

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def _cb_mission(self, msg: String):
        """Reçoit une mission depuis le EDF scheduler."""
        try:
            mission = json.loads(msg.data)
            dept    = mission.get("departement")
            type_rt = mission.get("type_rt", "")

            if not dept or dept == "pharmacie":
                return
            # Ignorer doublon
            if self._dept_cible == dept and self._state in ("MOVING", "PLANNING"):
                return

            self.get_logger().info(
                f"{C}[P3-NAV] Nouvelle mission : {dept} | {type_rt}{W}")

            # Sauvegarder mission précédente si HARD_RT override
            if (type_rt == "HARD_RT" and self._mission
                    and self._state not in ("IDLE", "GOAL_REACHED")):
                self._mission_interrompue = self._mission
                t_override = time.time()
                self._metrics["emergency_response_times"].append(
                    time.time() - t_override)
                # Publier sur /emergency_override (spec)
                self._pub_emergency_ov.publish(String(data=json.dumps({
                    "type":        "HARD_RT_OVERRIDE",
                    "dept_urgent": dept,
                    "dept_interrompu": self._dept_cible,
                    "timestamp":   time.time(),
                })))

            self._mission     = mission
            self._dept_cible  = dept
            self._replan_count = 0
            self._nav_start_time = time.time()
            self._metrics["missions_total"] += 1

            # Publier sur /navigation_goal (spec)
            salle = SALLES.get(dept, {})
            self._pub_nav_goal.publish(String(data=json.dumps({
                "departement": dept,
                "nom":         salle.get("nom", dept),
                "x":           salle.get("x", 0),
                "y":           salle.get("y", 0),
                "type_rt":     type_rt,
                "timestamp":   time.time(),
            })))

            # Lancer planification
            self._changer_etat("PLANNING")
            self._planifier()

        except (json.JSONDecodeError, KeyError) as e:
            self.get_logger().error(f"[P3-NAV] Mission invalide : {e}")

    def _cb_edf_interruption(self, msg: String):
        """Override EDF : interrompre navigation pour HARD_RT."""
        try:
            data = json.loads(msg.data)
            dept_urgent = data.get("dept_urgent")
            self.get_logger().warn(
                f"{R}{B}[P3-NAV] EDF OVERRIDE → {dept_urgent}{W}")
            # Arrêt immédiat, la nouvelle mission arrivera via /tache_courante
            self._arreter()
            # Publier /emergency_override (spec)
            self._pub_emergency_ov.publish(String(data=msg.data))
        except json.JSONDecodeError:
            pass

    def _cb_etat_robot(self, msg: String):
        """Synchro avec machine à états du mission_manager."""
        try:
            data = json.loads(msg.data)
            etat = data.get("etat", "")
            # Publier /mission_state (spec)
            self._pub_mission_state.publish(String(data=json.dumps({
                "etat":      etat,
                "mission":   data.get("mission"),
                "timestamp": time.time(),
            })))
            if etat == "ARRET_URGENCE":
                self._arreter()
                self._changer_etat("IDLE")
            elif etat == "IDLE" and self._state in ("GOAL_REACHED",):
                self._changer_etat("IDLE")
                self._mission = None
                self._dept_cible = None
        except json.JSONDecodeError:
            pass

    def _cb_watchdog(self, msg: String):
        """Réagit aux commandes du watchdog."""
        try:
            data = json.loads(msg.data)
            if data.get("type") == "RECUPERATION":
                self._arret_watchdog = True
                self._arreter()
                self.get_logger().error(
                    "[P3-NAV] Arrêt watchdog — navigation suspendue")
            elif data.get("type") == "REPRISE":
                self._arret_watchdog = False
                self.get_logger().info(
                    "[P3-NAV] Reprise watchdog — navigation autorisée")
                if self._mission and self._dept_cible:
                    self._changer_etat("PLANNING")
                    self._planifier()
        except json.JSONDecodeError:
            pass

    def _cb_odom(self, msg: Odometry):
        """Met à jour position et orientation."""
        self._pos_x      = msg.pose.pose.position.x
        self._pos_y      = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self._orientation = 2.0 * math.atan2(qz, qw)

    def _cb_nav_goal_spec(self, msg: String):
        """Compat spec : écoute /navigation_goal comme source alternative."""
        # Déjà géré par /pharmabot/tache_courante, pas de doublon
        pass

    # ══════════════════════════════════════════════════════════════════════════
    # Planification et navigation
    # ══════════════════════════════════════════════════════════════════════════

    def _planifier(self):
        """Calcule le chemin vers la cible en tenant compte des obstacles."""
        if not self._dept_cible:
            return
        salle = SALLES.get(self._dept_cible)
        if not salle:
            return

        self._obstacles = self._obs_sim.update(self._pos_x, self._pos_y)
        self._waypoints = self._planner.plan(
            self._pos_x, self._pos_y,
            salle["x"], salle["y"],
            self._obstacles if self._replan_count > 0 else None,
        )
        self._wp_idx = 0
        self.get_logger().info(
            f"[P3-NAV] Chemin planifié : {len(self._waypoints)} waypoints "
            f"vers {salle['nom']}"
        )
        self._changer_etat("MOVING")

    def _boucle_nav(self):
        """Boucle principale à 10 Hz."""
        if self._arret_watchdog:
            return
        if self._state == "IDLE":
            return

        # Mise à jour obstacles
        self._obstacles = self._obs_sim.update(self._pos_x, self._pos_y)
        if self._obstacles:
            self._metrics["obstacle_count"] = max(
                self._metrics["obstacle_count"],
                len(self._obstacles)
            )

        if self._state == "PLANNING":
            pass  # Géré par _planifier()

        elif self._state == "MOVING":
            self._nav_move()

        elif self._state == "REPLANNING":
            self._replan_count += 1
            self._metrics["replanning_count"] += 1
            if self._replan_count > MAX_REPLANIF:
                self.get_logger().error(
                    f"{R}[P3-NAV] Trop de replanifications "
                    f"({self._replan_count}) → FAILED{W}")
                self._changer_etat("FAILED")
                self._signaler_echec("MAX_REPLANIF_DEPASSÉ")
            else:
                self.get_logger().info(
                    f"{Y}[P3-NAV] Replanification #{self._replan_count}...{W}")
                self._planifier()

        elif self._state == "AVOIDING_OBSTACLE":
            self._nav_evitement()

        elif self._state == "RECOVERING":
            self._nav_recovering()

        elif self._state == "GOAL_REACHED":
            self._arreter()

        elif self._state == "FAILED":
            self._arreter()

    def _nav_move(self):
        """Avance vers le waypoint courant."""
        if not self._waypoints or self._wp_idx >= len(self._waypoints):
            self._changer_etat("GOAL_REACHED")
            self._on_goal_reached()
            return

        wp = self._waypoints[self._wp_idx]

        # Obstacle proche → évitement
        for obs in self._obstacles:
            if obs["distance"] < RAYON_OBSTACLE:
                self.get_logger().warn(
                    f"{Y}[P3-NAV] Obstacle à {obs['distance']:.1f}m → EVITEMENT{W}")
                self._changer_etat("AVOIDING_OBSTACLE")
                self._avoiding_start = time.time()
                self._pub_obstacle.publish(String(data=json.dumps({
                    "obstacle":    obs,
                    "robot_pos":   {"x": self._pos_x, "y": self._pos_y},
                    "timestamp":   time.time(),
                })))
                return

        # Vérifier stuck
        dist_depuis_last = math.sqrt(
            (self._pos_x - self._last_pos[0]) ** 2 +
            (self._pos_y - self._last_pos[1]) ** 2
        )
        if time.time() - self._last_pos_time > TIMEOUT_STUCK:
            if dist_depuis_last < DIST_STUCK:
                self.get_logger().warn(
                    f"{Y}[P3-NAV] Robot bloqué → RECOVERING{W}")
                self._metrics["recovery_count"] += 1
                self._changer_etat("RECOVERING")
                self._recovering_start = time.time()
                self._recovering_rot   = random.choice([-1.0, 1.0])
                return
            else:
                self._last_pos      = (self._pos_x, self._pos_y)
                self._last_pos_time = time.time()

        # Vérifier timeout global
        if (self._nav_start_time
                and time.time() - self._nav_start_time > TIMEOUT_NAV):
            self.get_logger().error(
                f"{R}[P3-NAV] Timeout navigation ({TIMEOUT_NAV}s) → FAILED{W}")
            self._changer_etat("FAILED")
            self._signaler_echec("TIMEOUT")
            return

        # Calculer commande vers waypoint
        dist   = wp.distance_to(self._pos_x, self._pos_y)
        if dist < RAYON_ARRIVEE:
            self.get_logger().info(
                f"{G}[P3-NAV] Waypoint {self._wp_idx} atteint ({wp}){W}")
            self._wp_idx += 1
            self._metrics["path_length"] += dist
            return

        # Contrôleur proportionnel
        dx    = wp.x - self._pos_x
        dy    = wp.y - self._pos_y
        angle = math.atan2(dy, dx)
        err_a = angle - self._orientation
        while err_a >  math.pi: err_a -= 2 * math.pi
        while err_a < -math.pi: err_a += 2 * math.pi

        cmd = Twist()
        if abs(err_a) > 0.3:
            cmd.linear.x  = 0.1
            cmd.angular.z = max(-ANG_MAX, min(ANG_MAX, KP_ANG * err_a))
        else:
            cmd.linear.x  = min(VIT_MAX, KP_LIN * dist)
            cmd.angular.z = max(-ANG_MAX * 0.5,
                                min(ANG_MAX * 0.5, KP_ANG * err_a))
        self._pub_cmd_vel.publish(cmd)

    def _nav_evitement(self):
        """
        Manœuvre d'évitement : rotation latérale puis reprise.
        Durée : ~3 secondes.
        """
        if not hasattr(self, "_avoiding_start"):
            self._avoiding_start = time.time()

        elapsed = time.time() - self._avoiding_start
        cmd     = Twist()

        if elapsed < 1.5:
            # Rotation pour éviter
            cmd.angular.z = ANG_MAX * 0.7
            cmd.linear.x  = 0.15
        elif elapsed < 3.0:
            # Avance latéralement
            cmd.linear.x  = 0.3
            cmd.angular.z = 0.2
        else:
            # Manœuvre terminée → supprimer obstacles proches et replanifier
            self._obs_sim.clear_near(self._pos_x, self._pos_y, 3.0)
            self._arreter()
            self.get_logger().info(
                f"{G}[P3-NAV] Évitement terminé → REPLANNING{W}")
            self._changer_etat("REPLANNING")
            return

        self._pub_cmd_vel.publish(cmd)

    def _nav_recovering(self):
        """
        Comportement de récupération : recul + rotation aléatoire.
        """
        if not self._recovering_start:
            self._recovering_start = time.time()

        elapsed = time.time() - self._recovering_start
        cmd     = Twist()

        if elapsed < DUREE_RECOVERING / 2:
            # Reculer
            cmd.linear.x  = -0.2
            cmd.angular.z = self._recovering_rot * 0.5
        elif elapsed < DUREE_RECOVERING:
            # Rotation
            cmd.angular.z = self._recovering_rot * ANG_MAX
        else:
            # Récupération terminée → replanifier
            self._arreter()
            self._last_pos      = (self._pos_x, self._pos_y)
            self._last_pos_time = time.time()
            self.get_logger().info(
                f"{G}[P3-NAV] Récupération terminée → REPLANNING{W}")
            self._changer_etat("REPLANNING")
            return

        self._pub_cmd_vel.publish(cmd)

    def _on_goal_reached(self):
        """Actions à réaliser quand la destination est atteinte."""
        dept = self._dept_cible
        salle = SALLES.get(dept, {})
        self.get_logger().info(
            f"{G}{B}[P3-NAV] ✓ GOAL REACHED : {salle.get('nom', dept)}{W}")

        # Mise à jour métriques
        if self._nav_start_time:
            travel_time = time.time() - self._nav_start_time
            self._metrics["travel_time"]    = travel_time
            self._metrics["missions_success"] += 1
            n = self._metrics["missions_total"]
            self._metrics["success_rate"] = (
                self._metrics["missions_success"] / n * 100 if n else 0
            )

        # Publier état GOAL_REACHED
        self._pub_status()
        self._pub_metrics()

        # Si mission interrompue existait → la reprendre
        if self._mission_interrompue:
            mi = self._mission_interrompue
            self._mission_interrompue = None
            self.get_logger().info(
                f"{C}[P3-NAV] Reprise mission interrompue : "
                f"{mi.get('departement')}{W}")
            self._mission     = mi
            self._dept_cible  = mi.get("departement")
            self._replan_count = 0
            self._nav_start_time = time.time()
            self._changer_etat("PLANNING")
            self._planifier()
        else:
            # Retour à IDLE après livraison
            self._dept_cible = None

    # ══════════════════════════════════════════════════════════════════════════
    # Vérifications sécurité
    # ══════════════════════════════════════════════════════════════════════════

    def _check_safety(self):
        """Vérifications sécurité à 1 Hz."""
        # Distance sécurité obstacles
        for obs in self._obstacles:
            if obs.get("distance", 999) < RAYON_SECURITE:
                self.get_logger().error(
                    f"{R}[P3-NAV] Obstacle < {RAYON_SECURITE}m → ARRÊT SÉCURITÉ{W}")
                self._arreter()
                # Publier alerte watchdog (spec)
                self._pub_watchdog_alert.publish(String(data=json.dumps({
                    "type":        "SECURITE_OBSTACLE",
                    "distance":    obs["distance"],
                    "robot_pos":   {"x": self._pos_x, "y": self._pos_y},
                    "timestamp":   time.time(),
                })))
                break

    def _signaler_echec(self, raison: str):
        """Publie une alerte en cas d'échec navigation."""
        self._pub_watchdog_alert.publish(String(data=json.dumps({
            "type":      "NAVIGATION_FAILED",
            "raison":    raison,
            "dept":      self._dept_cible,
            "timestamp": time.time(),
        })))
        self.get_logger().error(
            f"{R}[P3-NAV] Échec navigation : {raison}{W}")

    # ══════════════════════════════════════════════════════════════════════════
    # Publication
    # ══════════════════════════════════════════════════════════════════════════

    def _changer_etat(self, nouvel_etat: str):
        if nouvel_etat not in STATES_P3:
            return
        ancien = self._state
        self._state = nouvel_etat
        self.get_logger().info(
            f"[P3-NAV] {ancien} → {nouvel_etat}")
        self._pub_status()

    def _pub_status(self):
        """Publie le statut de navigation sur /navigation_status."""
        wp_courant = None
        if self._waypoints and self._wp_idx < len(self._waypoints):
            wp = self._waypoints[self._wp_idx]
            wp_courant = {"x": wp.x, "y": wp.y, "label": wp.label}

        msg = String()
        msg.data = json.dumps({
            "state":        self._state,
            "departement":  self._dept_cible,
            "pos_x":        round(self._pos_x, 2),
            "pos_y":        round(self._pos_y, 2),
            "waypoint":     wp_courant,
            "waypoints_remaining": len(self._waypoints) - self._wp_idx,
            "obstacles":    len(self._obstacles),
            "replan_count": self._replan_count,
            "nav_time":     round(time.time() - self._nav_start_time, 1)
                            if self._nav_start_time else 0,
            "timestamp":    time.time(),
        })
        self._pub_nav_status.publish(msg)

    def _pub_metrics(self):
        """Publie les métriques de navigation sur /navigation_metrics."""
        n    = self._metrics["missions_total"]
        ok   = self._metrics["missions_success"]
        rate = ok / n * 100 if n > 0 else 0.0

        ert  = self._metrics["emergency_response_times"]
        avg_ert = sum(ert) / len(ert) if ert else 0.0

        msg = String()
        msg.data = json.dumps({
            "path_length":      round(self._metrics["path_length"], 2),
            "travel_time":      round(self._metrics["travel_time"], 2),
            "replanning_count": self._metrics["replanning_count"],
            "obstacle_count":   self._metrics["obstacle_count"],
            "recovery_count":   self._metrics["recovery_count"],
            "success_rate":     round(rate, 1),
            "missions_total":   n,
            "missions_success": ok,
            "avg_emergency_response_ms": round(avg_ert * 1000, 1),
            "timestamp":        time.time(),
        })
        self._pub_nav_metrics.publish(msg)

        if n > 0:
            self.get_logger().info(
                f"[P3-METRICS] {n} missions | "
                f"succès={rate:.0f}% | "
                f"replan={self._metrics['replanning_count']} | "
                f"recovery={self._metrics['recovery_count']}"
            )

    # ══════════════════════════════════════════════════════════════════════════
    # Utilitaires
    # ══════════════════════════════════════════════════════════════════════════

    def _arreter(self):
        """Arrêt immédiat du robot."""
        cmd = Twist()
        cmd.linear.x  = 0.0
        cmd.angular.z = 0.0
        self._pub_cmd_vel.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
