#!/usr/bin/env python3
"""
navigation_watchdog_p3.py — Moniteur de Sécurité Navigation Phase 3
PharmaBot | Ibn Tofaïl University | 2025-2026
Prof. Khaoula Boukir — Systèmes Embarqués Temps Réel

Phase 3 : Watchdog spécialisé pour la navigation autonome.
Complémentaire au watchdog_node.py existant (ne le remplace pas).

Détections :
    ✓ Robot bloqué (position stagnante > TIMEOUT_STUCK)
    ✓ Perte de localisation (pas d'odométrie > TIMEOUT_ODOM)
    ✓ Échec planificateur (PLANNING trop longtemps)
    ✓ Timeout de navigation globale (> TIMEOUT_NAV)
    ✓ Déviation excessive par rapport au chemin prévu
    ✓ Obstacles persistants (robot ne progresse pas)

Alertes publiées :
    /watchdog_alert             → alertes spec Phase 3
    /pharmabot/watchdog         → compat watchdog_node.py existant

Métriques :
    /navigation_metrics         → agrégation avec autonomous_navigator
"""

import json
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# ──────────────────────────────────────────────────────────────────────────────
RESET = "\033[0m"
BOLD  = "\033[1m"
RED   = "\033[91m"
GREEN = "\033[92m"
YELL  = "\033[93m"
CYAN  = "\033[96m"

# Seuils de détection
TIMEOUT_STUCK     = 15.0   # s — robot considéré bloqué
DIST_STUCK        = 0.25   # m — déplacement min pour ne pas être "stuck"
TIMEOUT_ODOM      = 5.0    # s — délai sans odométrie → perte localisation
TIMEOUT_PLANNING  = 10.0   # s — temps max en PLANNING avant alerte
TIMEOUT_NAV_TOTAL = 180.0  # s — timeout global d'une mission
DEVIATION_MAX     = 8.0    # m — déviation max par rapport au chemin direct
WINDOW_HISTORY    = 30     # points d'historique position


class NavigationWatchdogP3(Node):
    """
    Moniteur de sécurité de navigation Phase 3.
    Surveille la navigation autonome et génère des alertes précises.
    """

    def __init__(self):
        super().__init__("navigation_watchdog_p3")
        self.get_logger().info(
            f"{BOLD}{CYAN}Navigation Watchdog Phase 3 démarré{RESET}")

        # ── État interne ──────────────────────────────────────────────────────
        self._pos_x          = 1.0
        self._pos_y          = 16.0
        self._pos_hist       = []    # historique positions (x, y, ts)
        self._last_odom_ts   = time.time()
        self._nav_state      = "IDLE"
        self._dept_cible     = None
        self._nav_start_ts   = None   # timestamp début mission
        self._planning_ts    = None   # timestamp entrée PLANNING
        self._mission_active = False

        # Suivi déviation
        self._goal_x = None
        self._goal_y = None

        # Stats watchdog P3
        self._stats = {
            "alertes_stuck":       0,
            "alertes_localisation":0,
            "alertes_planning":    0,
            "alertes_timeout":     0,
            "alertes_deviation":   0,
            "total_alertes":       0,
            "recups_nav":          0,
        }

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_alert    = self.create_publisher(
            String, "/watchdog_alert",     10)
        self._pub_watchdog = self.create_publisher(
            String, "/pharmabot/watchdog", 10)   # compat existant

        # ── Subscribers ───────────────────────────────────────────────────────
        self._sub_odom = self.create_subscription(
            Odometry, "/demo/odom",
            self._cb_odom, 1)
        self._sub_nav_status = self.create_subscription(
            String, "/navigation_status",
            self._cb_nav_status, 10)
        self._sub_nav_goal = self.create_subscription(
            String, "/navigation_goal",
            self._cb_nav_goal, 10)
        self._sub_obstacle = self.create_subscription(
            String, "/pharmabot/obstacle_detected",
            self._cb_obstacle, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self._timer_check  = self.create_timer(2.0, self._check_all)
        self._timer_report = self.create_timer(30.0, self._rapport_stats)

        self.get_logger().info(
            f"{GREEN}Navigation Watchdog P3 prêt — "
            f"stuck={TIMEOUT_STUCK}s "
            f"odom={TIMEOUT_ODOM}s "
            f"timeout={TIMEOUT_NAV_TOTAL}s{RESET}")

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks
    # ══════════════════════════════════════════════════════════════════════════

    def _cb_odom(self, msg: Odometry):
        """Met à jour position et timestamp odométrie."""
        self._pos_x       = msg.pose.pose.position.x
        self._pos_y       = msg.pose.pose.position.y
        self._last_odom_ts = time.time()

        # Historique positions
        self._pos_hist.append((self._pos_x, self._pos_y, time.time()))
        if len(self._pos_hist) > WINDOW_HISTORY:
            self._pos_hist.pop(0)

    def _cb_nav_status(self, msg: String):
        """Synchro avec l'état du autonomous_navigator."""
        try:
            data = json.loads(msg.data)
            old_state   = self._nav_state
            self._nav_state  = data.get("state", "IDLE")
            self._dept_cible = data.get("departement")

            # Détecter entrée en PLANNING
            if self._nav_state == "PLANNING" and old_state != "PLANNING":
                self._planning_ts = time.time()

            # Détecter début mission
            if (self._nav_state not in ("IDLE",) and not self._mission_active):
                self._mission_active = True
                self._nav_start_ts   = time.time()
                self.get_logger().info(
                    f"[WD-P3] Mission démarrée : {self._dept_cible}")

            # Détecter fin mission
            if self._nav_state in ("GOAL_REACHED", "IDLE") and self._mission_active:
                self._mission_active = False
                self._nav_start_ts   = None
                self._planning_ts    = None
                if self._nav_state == "GOAL_REACHED":
                    self._stats["recups_nav"] += 1  # mission réussie

        except json.JSONDecodeError:
            pass

    def _cb_nav_goal(self, msg: String):
        """Mémorise le goal pour calcul déviation."""
        try:
            data = json.loads(msg.data)
            self._goal_x = data.get("x")
            self._goal_y = data.get("y")
        except json.JSONDecodeError:
            pass

    def _cb_obstacle(self, msg: String):
        """Reçoit les notifications d'obstacle du autonomous_navigator."""
        try:
            data = json.loads(msg.data)
            obs  = data.get("obstacle", {})
            dist = obs.get("distance", 999)
            self.get_logger().warn(
                f"{YELL}[WD-P3] Obstacle détecté à {dist:.1f}m{RESET}")
        except json.JSONDecodeError:
            pass

    # ══════════════════════════════════════════════════════════════════════════
    # Vérifications principales
    # ══════════════════════════════════════════════════════════════════════════

    def _check_all(self):
        """Effectue toutes les vérifications toutes les 2s."""
        if not self._mission_active:
            return

        self._check_stuck()
        self._check_odom_loss()
        self._check_planning_timeout()
        self._check_nav_timeout()
        self._check_deviation()

    def _check_stuck(self):
        """
        Détecte si le robot n'a pas bougé depuis TIMEOUT_STUCK secondes.
        """
        if len(self._pos_hist) < 5:
            return
        # Comparer position actuelle vs il y a TIMEOUT_STUCK secondes
        now = time.time()
        old_pts = [p for p in self._pos_hist if now - p[2] >= TIMEOUT_STUCK]
        if not old_pts:
            return
        old_x, old_y, _ = old_pts[0]
        dist = math.sqrt((self._pos_x - old_x) ** 2 + (self._pos_y - old_y) ** 2)

        if dist < DIST_STUCK and self._nav_state == "MOVING":
            self._stats["alertes_stuck"] += 1
            self._stats["total_alertes"] += 1
            self._publier_alerte("ROBOT_STUCK", {
                "distance_parcourue_m": round(dist, 3),
                "duree_blocage_s":      TIMEOUT_STUCK,
                "pos":                  {"x": self._pos_x, "y": self._pos_y},
            })
            self.get_logger().warn(
                f"{YELL}[WD-P3] Robot bloqué ! "
                f"Déplacement={dist:.3f}m en {TIMEOUT_STUCK}s{RESET}")

    def _check_odom_loss(self):
        """Détecte la perte d'odométrie (pas de message /demo/odom)."""
        delai = time.time() - self._last_odom_ts
        if delai > TIMEOUT_ODOM:
            self._stats["alertes_localisation"] += 1
            self._stats["total_alertes"] += 1
            self._publier_alerte("ODOM_LOSS", {
                "delai_odom_s": round(delai, 1),
                "derniere_pos": {"x": self._pos_x, "y": self._pos_y},
            })
            self.get_logger().error(
                f"{RED}[WD-P3] Perte odométrie ! "
                f"Dernier message il y a {delai:.1f}s{RESET}")

    def _check_planning_timeout(self):
        """Détecte un planificateur bloqué."""
        if self._nav_state != "PLANNING" or not self._planning_ts:
            return
        elapsed = time.time() - self._planning_ts
        if elapsed > TIMEOUT_PLANNING:
            self._stats["alertes_planning"] += 1
            self._stats["total_alertes"] += 1
            self._publier_alerte("PLANNER_FAILURE", {
                "duree_planning_s": round(elapsed, 1),
                "dept_cible":       self._dept_cible,
            })
            self.get_logger().error(
                f"{RED}[WD-P3] Planificateur bloqué depuis {elapsed:.1f}s !{RESET}")

    def _check_nav_timeout(self):
        """Détecte un timeout global de navigation."""
        if not self._nav_start_ts:
            return
        elapsed = time.time() - self._nav_start_ts
        if elapsed > TIMEOUT_NAV_TOTAL:
            self._stats["alertes_timeout"] += 1
            self._stats["total_alertes"] += 1
            self._publier_alerte("NAV_TIMEOUT", {
                "duree_mission_s": round(elapsed, 1),
                "dept_cible":      self._dept_cible,
                "etat_nav":        self._nav_state,
            })
            self.get_logger().error(
                f"{RED}[WD-P3] Timeout navigation : "
                f"{elapsed:.0f}s > {TIMEOUT_NAV_TOTAL}s{RESET}")

    def _check_deviation(self):
        """Détecte une déviation excessive par rapport à la destination."""
        if self._goal_x is None or self._goal_y is None:
            return
        if self._nav_state not in ("MOVING", "AVOIDING_OBSTACLE"):
            return

        # Distance directe attendue (début mission → goal)
        dist_goal = math.sqrt(
            (self._goal_x - self._pos_x) ** 2 +
            (self._goal_y - self._pos_y) ** 2
        )
        # Vérifier si le robot s'éloigne du but
        if len(self._pos_hist) >= 10:
            old_x, old_y, _ = self._pos_hist[-10]
            dist_old = math.sqrt(
                (self._goal_x - old_x) ** 2 +
                (self._goal_y - old_y) ** 2
            )
            deviation = dist_goal - dist_old  # positif = robot s'éloigne
            if deviation > DEVIATION_MAX:
                self._stats["alertes_deviation"] += 1
                self._stats["total_alertes"] += 1
                self._publier_alerte("PATH_DEVIATION", {
                    "deviation_m":   round(deviation, 2),
                    "dist_goal_m":   round(dist_goal, 2),
                    "dept_cible":    self._dept_cible,
                })
                self.get_logger().warn(
                    f"{YELL}[WD-P3] Déviation chemin : "
                    f"+{deviation:.1f}m du goal{RESET}")

    # ══════════════════════════════════════════════════════════════════════════
    # Publication alertes
    # ══════════════════════════════════════════════════════════════════════════

    def _publier_alerte(self, type_alerte: str, details: dict):
        """Publie une alerte sur /watchdog_alert ET /pharmabot/watchdog."""
        payload = json.dumps({
            "source":     "navigation_watchdog_p3",
            "type_alerte": type_alerte,
            "details":    details,
            "nav_state":  self._nav_state,
            "dept_cible": self._dept_cible,
            "timestamp":  time.time(),
        })
        # Topic spec Phase 3
        self._pub_alert.publish(String(data=payload))
        # Topic compat Phase 1+2
        self._pub_watchdog.publish(String(data=json.dumps({
            "type":    "ALERTE_NAV_P3",
            "raison":  type_alerte,
            "details": details,
            "timestamp": time.time(),
        })))

    def _rapport_stats(self):
        """Rapport stats toutes les 30s."""
        n = self._stats["total_alertes"]
        self.get_logger().info(
            f"[WD-P3] Stats — Total alertes: {n} | "
            f"stuck={self._stats['alertes_stuck']} | "
            f"odom={self._stats['alertes_localisation']} | "
            f"planning={self._stats['alertes_planning']} | "
            f"timeout={self._stats['alertes_timeout']}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationWatchdogP3()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
