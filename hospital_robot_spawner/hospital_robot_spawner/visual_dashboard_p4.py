#!/usr/bin/env python3
"""
visual_dashboard_p4.py — Dashboard Visuel Avancé Phase 4
PharmaBot | Ibn Tofaïl University | 2025-2026
Prof. Khaoula Boukir — Systèmes Embarqués Temps Réel

PHASE 4 : Dashboard complet agrégant TOUS les topics Phase 1+2+3+4.
Affichage terminal avec sections :
    1. 🗓  SCHEDULER — mission active, file EDF, deadlines, priorités
    2. 🧭  NAVIGATION — position, chemin, obstacles, ETA, état P3
    3. 📦  LIVRAISONS — historique, taux succès, échecs
    4. 🔒  SÉCURITÉ  — alertes watchdog, urgences, humains
    5. 👁  PERCEPTION — salles détectées, confirmations visuelles

Compatible avec les 20+ topics existants Phase 1+2+3+4.

Usage :
    ros2 run hospital_robot_spawner visual_dashboard_p4
"""

import json
import math
import time
import os
import sys
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

# ──────────────────────────────────────────────────────────────────────────────
# Couleurs et styles
# ──────────────────────────────────────────────────────────────────────────────
R  = "\033[91m"; G  = "\033[92m"; Y  = "\033[93m"
B  = "\033[94m"; M  = "\033[95m"; C  = "\033[96m"
W  = "\033[0m";  BD = "\033[1m";  DIM = "\033[2m"
BG_DARK = "\033[40m"

# Largeur du dashboard
WIDTH = 80


def box_line(char: str = "─", width: int = WIDTH) -> str:
    return char * width


def header(title: str, color: str = C) -> str:
    pad = (WIDTH - len(title) - 4) // 2
    return f"{color}{BD}{'─'*pad}  {title}  {'─'*(WIDTH-pad-len(title)-4)}{W}"


def fmt_time(ts: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(ts)) if ts else "--:--:--"


def fmt_rt(type_rt: str) -> str:
    if type_rt == "HARD_RT":
        return f"{R}{BD}⚡ HARD_RT{W}"
    if type_rt == "SOFT_RT":
        return f"{Y}{BD}⚠ SOFT_RT{W}"
    return f"{G}FIRM_RT{W}"


def badge(label: str, color: str) -> str:
    return f"{color}{BD}[{label}]{W}"


class VisualDashboardP4(Node):
    """
    Dashboard visuel Phase 4 — Agrège tous les topics PharmaBot.
    """

    def __init__(self):
        super().__init__("visual_dashboard_p4")
        self.get_logger().info(
            f"{BD}{C}Visual Dashboard Phase 4 démarré{W}")

        self._lock = threading.Lock()

        # ── Données internes ──────────────────────────────────────────────────

        # Section Scheduler
        self._tache_courante      = None
        self._alerte_rt           = None
        self._stats_scheduler     = {}
        self._queue_count         = 0

        # Section Navigation P3
        self._nav_status          = None
        self._nav_metrics         = None
        self._nav_state_p3        = "IDLE"
        self._pos_x               = 1.0
        self._pos_y               = 16.0
        self._dept_cible_nav      = None
        self._obstacle_count      = 0
        self._nav_start_ts        = None

        # Section Mission / État robot
        self._etat_robot          = None
        self._mission_log         = []      # last 10
        self._dma_etat            = "IDLE"

        # Section Livraisons
        self._livraisons          = []      # historique complet
        self._delivery_stats      = {}
        self._dernier_type_rt     = "?"
        self._nb_deadlines_ok     = 0
        self._nb_deadlines_ko     = 0

        # Section Sécurité
        self._watchdog_alertes    = []      # last 5
        self._watchdog_alert_p3   = []      # last 5
        self._emergency_events    = []
        self._nb_overrides        = 0

        # Section Perception P4
        self._room_detected       = None
        self._delivery_confirmed_visual = []
        self._human_detected      = None
        self._perception_stats    = {}

        # ── Publishers ────────────────────────────────────────────────────────
        # Dashboard publie aussi un résumé JSON
        self._pub_summary = self.create_publisher(
            String, "/pharmabot/dashboard_summary", 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        # Phase 1+2
        self._sub_tache   = self.create_subscription(
            String, "/pharmabot/tache_courante",
            self._cb_tache, 10)
        self._sub_alerte  = self.create_subscription(
            String, "/pharmabot/alerte_rt",
            self._cb_alerte_rt, 10)
        self._sub_etat    = self.create_subscription(
            String, "/pharmabot/etat_robot",
            self._cb_etat_robot, 10)
        self._sub_lv      = self.create_subscription(
            String, "/pharmabot/livraison_confirmee",
            self._cb_livraison, 10)
        self._sub_stats   = self.create_subscription(
            String, "/pharmabot/delivery_stats",
            self._cb_delivery_stats, 10)
        self._sub_wd      = self.create_subscription(
            String, "/pharmabot/watchdog",
            self._cb_watchdog, 10)
        self._sub_dma     = self.create_subscription(
            String, "/pharmabot/dma_pipeline",
            self._cb_dma, 10)
        self._sub_log     = self.create_subscription(
            String, "/pharmabot/mission_log",
            self._cb_log, 10)
        self._sub_odom    = self.create_subscription(
            Odometry, "/demo/odom",
            self._cb_odom, 1)
        self._sub_nav_bridge = self.create_subscription(
            String, "/pharmabot/nav_status",
            self._cb_nav_bridge, 10)

        # Phase 3 (nouveaux)
        self._sub_nav_p3    = self.create_subscription(
            String, "/navigation_status",
            self._cb_nav_status_p3, 10)
        self._sub_metrics   = self.create_subscription(
            String, "/navigation_metrics",
            self._cb_nav_metrics, 10)
        self._sub_obstacle  = self.create_subscription(
            String, "/pharmabot/obstacle_detected",
            self._cb_obstacle, 10)
        self._sub_nav_goal  = self.create_subscription(
            String, "/navigation_goal",
            self._cb_nav_goal, 10)
        self._sub_wd_alert  = self.create_subscription(
            String, "/watchdog_alert",
            self._cb_watchdog_alert_p3, 10)
        self._sub_emergency = self.create_subscription(
            String, "/emergency_override",
            self._cb_emergency, 10)

        # Phase 4 (nouveaux)
        self._sub_room    = self.create_subscription(
            String, "/perception/room_detected",
            self._cb_room, 10)
        self._sub_dlv_vis = self.create_subscription(
            String, "/perception/delivery_confirmed",
            self._cb_delivery_visual, 10)
        self._sub_human   = self.create_subscription(
            String, "/perception/human_detected",
            self._cb_human, 10)
        self._sub_dlv_sts = self.create_subscription(
            String, "/delivery_status",
            self._cb_delivery_status, 10)
        self._sub_pharm   = self.create_subscription(
            String, "/pharmacy_validation",
            self._cb_pharmacy_validation, 10)

        # ── Timers ────────────────────────────────────────────────────────────
        self._timer_display = self.create_timer(2.0, self._afficher)
        self._timer_summary = self.create_timer(10.0, self._publier_summary)

        self.get_logger().info(f"{G}Visual Dashboard P4 prêt — "
                               f"actualisation 2s{W}")

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks Phase 1+2
    # ══════════════════════════════════════════════════════════════════════════

    def _cb_tache(self, msg: String):
        try:
            with self._lock:
                self._tache_courante = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_alerte_rt(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._alerte_rt = data
        except json.JSONDecodeError:
            pass

    def _cb_etat_robot(self, msg: String):
        try:
            with self._lock:
                self._etat_robot = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_livraison(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._livraisons.append(data)
                if len(self._livraisons) > 50:
                    self._livraisons.pop(0)
                if data.get("deadline_ok"):
                    self._nb_deadlines_ok += 1
                else:
                    self._nb_deadlines_ko += 1
                self._dernier_type_rt = data.get("type_rt", "?")
        except json.JSONDecodeError:
            pass

    def _cb_delivery_stats(self, msg: String):
        try:
            with self._lock:
                self._delivery_stats = json.loads(msg.data).get("stats", {})
        except json.JSONDecodeError:
            pass

    def _cb_watchdog(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._watchdog_alertes.append(data)
                if len(self._watchdog_alertes) > 5:
                    self._watchdog_alertes.pop(0)
        except json.JSONDecodeError:
            pass

    def _cb_dma(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._dma_etat = data.get("etat", "?")
        except json.JSONDecodeError:
            pass

    def _cb_log(self, msg: String):
        try:
            with self._lock:
                self._mission_log.append(msg.data[:60])
                if len(self._mission_log) > 10:
                    self._mission_log.pop(0)
        except Exception:
            pass

    def _cb_odom(self, msg: Odometry):
        with self._lock:
            self._pos_x = msg.pose.pose.position.x
            self._pos_y = msg.pose.pose.position.y

    def _cb_nav_bridge(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._dept_cible_nav = data.get("departement_cible")
        except json.JSONDecodeError:
            pass

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks Phase 3
    # ══════════════════════════════════════════════════════════════════════════

    def _cb_nav_status_p3(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._nav_status    = data
                self._nav_state_p3  = data.get("state", "IDLE")
                self._obstacle_count = data.get("obstacles", 0)
        except json.JSONDecodeError:
            pass

    def _cb_nav_metrics(self, msg: String):
        try:
            with self._lock:
                self._nav_metrics = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_obstacle(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._obstacle_count += 1
        except json.JSONDecodeError:
            pass

    def _cb_nav_goal(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._dept_cible_nav = data.get("departement")
        except json.JSONDecodeError:
            pass

    def _cb_watchdog_alert_p3(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._watchdog_alert_p3.append(data)
                if len(self._watchdog_alert_p3) > 5:
                    self._watchdog_alert_p3.pop(0)
        except json.JSONDecodeError:
            pass

    def _cb_emergency(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._emergency_events.append(data)
                if len(self._emergency_events) > 10:
                    self._emergency_events.pop(0)
                self._nb_overrides += 1
        except json.JSONDecodeError:
            pass

    # ══════════════════════════════════════════════════════════════════════════
    # Callbacks Phase 4
    # ══════════════════════════════════════════════════════════════════════════

    def _cb_room(self, msg: String):
        try:
            with self._lock:
                self._room_detected = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_delivery_visual(self, msg: String):
        try:
            data = json.loads(msg.data)
            with self._lock:
                self._delivery_confirmed_visual.append(data)
                if len(self._delivery_confirmed_visual) > 20:
                    self._delivery_confirmed_visual.pop(0)
        except json.JSONDecodeError:
            pass

    def _cb_human(self, msg: String):
        try:
            with self._lock:
                self._human_detected = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _cb_delivery_status(self, msg: String):
        pass  # Déjà traité via livraison_confirmee

    def _cb_pharmacy_validation(self, msg: String):
        pass  # Logging uniquement

    # ══════════════════════════════════════════════════════════════════════════
    # Affichage Dashboard
    # ══════════════════════════════════════════════════════════════════════════

    def _afficher(self):
        """Affiche le dashboard complet toutes les 2s."""
        with self._lock:
            tache      = self._tache_courante
            etat_robot = self._etat_robot
            nav        = self._nav_status
            metrics    = self._nav_metrics
            livraisons = list(self._livraisons)
            wd_alts    = list(self._watchdog_alertes)
            wd_p3      = list(self._watchdog_alert_p3)
            room       = self._room_detected
            human      = self._human_detected
            vis_lvs    = list(self._delivery_confirmed_visual)
            emergencies = list(self._emergency_events)
            nb_ok      = self._nb_deadlines_ok
            nb_ko      = self._nb_deadlines_ko
            nb_ovr     = self._nb_overrides
            pos_x      = self._pos_x
            pos_y      = self._pos_y
            nav_state  = self._nav_state_p3
            dept_cible = self._dept_cible_nav
            dma_etat   = self._dma_etat
            obs_count  = self._obstacle_count

        os.system("clear")
        lines = []

        # ─── TITRE ────────────────────────────────────────────────────────────
        lines.append(f"\n{BD}{C}{'═'*WIDTH}{W}")
        lines.append(f"{BD}{C}{'PharmaBot — Dashboard Complet Phase 4':^{WIDTH}}{W}")
        lines.append(f"{DIM}{'Ibn Tofaïl University | Prof. Boukir | ' + fmt_time(time.time()):^{WIDTH}}{W}")
        lines.append(f"{BD}{C}{'═'*WIDTH}{W}")

        # ─── SECTION 1 : SCHEDULER ────────────────────────────────────────────
        lines.append(header("🗓  SCHEDULER EDF", Y))
        if tache:
            dept     = tache.get("departement", "?")
            type_rt  = tache.get("type_rt", "?")
            med      = tache.get("medicament", "?")
            dl       = tache.get("deadline_sec", 0)
            ts       = tache.get("timestamp", time.time())
            restant  = max(0, dl - (time.time() - ts))
            color_dl = R if restant < 10 else (Y if restant < 30 else G)
            lines.append(
                f"  Mission active : {BD}{dept}{W} | {fmt_rt(type_rt)} | "
                f"{med[:30]}")
            lines.append(
                f"  Deadline       : {color_dl}{BD}{restant:.0f}s{W} restantes "
                f"({dl}s total) | "
                f"DMA: {BD}{dma_etat}{W}")
        else:
            lines.append(f"  {DIM}Aucune mission active — EDF Scheduler en attente{W}")

        lines.append(
            f"  Livraisons     : "
            f"{G}✓ {nb_ok} deadlines OK{W}  "
            f"{R}✗ {nb_ko} dépassées{W}  "
            f"| EDF Overrides : {BD}{nb_ovr}{W}"
        )

        # ─── SECTION 2 : NAVIGATION ───────────────────────────────────────────
        lines.append(header("🧭  NAVIGATION AUTONOME (Phase 3)", B))
        # État robot
        if etat_robot:
            etat = etat_robot.get("etat", "?")
            col  = (G if etat in ("NAVIGATION","LIVRAISON")
                    else R if etat in ("ARRET_URGENCE","URGENCE_OVERRIDE")
                    else Y)
            lines.append(
                f"  État machine   : {col}{BD}{etat}{W}  |  "
                f"Nav P3 : {BD}{nav_state}{W}")
        else:
            lines.append(f"  État machine   : {DIM}en attente...{W}  |  Nav P3 : {BD}{nav_state}{W}")

        lines.append(
            f"  Position robot : "
            f"x={BD}{pos_x:+.1f}m{W}  y={BD}{pos_y:+.1f}m{W}"
            + (f"  → Cible : {BD}{dept_cible}{W}" if dept_cible else "")
        )

        if nav:
            wp   = nav.get("waypoint")
            rem  = nav.get("waypoints_remaining", 0)
            ntime = nav.get("nav_time", 0)
            lines.append(
                f"  Waypoints      : {rem} restants"
                + (f" | Prochain : ({wp['x']:.1f},{wp['y']:.1f})" if wp else "")
                + f" | Temps nav : {ntime:.0f}s"
            )

        if metrics:
            sr    = metrics.get("success_rate", 0)
            repl  = metrics.get("replanning_count", 0)
            recov = metrics.get("recovery_count", 0)
            ert   = metrics.get("avg_emergency_response_ms", 0)
            color_sr = G if sr >= 95 else (Y if sr >= 80 else R)
            lines.append(
                f"  Métriques      : "
                f"succès={color_sr}{BD}{sr:.0f}%{W} | "
                f"replan={BD}{repl}{W} | "
                f"recovery={BD}{recov}{W} | "
                f"ERT={BD}{ert:.0f}ms{W}"
            )

        lines.append(
            f"  Obstacles      : {BD}{obs_count}{W} détectés"
        )

        # ─── SECTION 3 : LIVRAISONS ───────────────────────────────────────────
        lines.append(header("📦  HISTORIQUE LIVRAISONS", G))
        total = nb_ok + nb_ko
        rate  = nb_ok / total * 100 if total else 0
        col_r = G if rate >= 95 else (Y if rate >= 80 else R)
        lines.append(
            f"  Total          : {BD}{total}{W} livraisons | "
            f"Taux succès : {col_r}{BD}{rate:.0f}%{W}"
        )
        if livraisons:
            for lv in livraisons[-4:]:
                dept  = lv.get("departement", "?")
                ok    = lv.get("deadline_ok", False)
                tl    = lv.get("temps_livraison", 0)
                dl    = lv.get("deadline_sec", 0)
                trt   = lv.get("type_rt", "?")
                num   = lv.get("livraison_num", "?")
                col   = G if ok else R
                icon  = "✓" if ok else "✗"
                lines.append(
                    f"  [{col}{icon}{W}] #{num} {dept[:12]:<12} "
                    f"{fmt_rt(trt)} "
                    f"{tl:.0f}s/{dl}s"
                )
        # Confirmations visuelles P4
        if vis_lvs:
            last_vis = vis_lvs[-1]
            acc = last_vis.get("accuracy", 0)
            lines.append(
                f"  Visuel P4      : "
                f"{BD}{len(vis_lvs)}{W} conf. visuelles | "
                f"dernière acc={G}{BD}{acc:.1%}{W} | "
                f"marker={last_vis.get('marker','?')}"
            )

        # ─── SECTION 4 : SÉCURITÉ ─────────────────────────────────────────────
        lines.append(header("🔒  SÉCURITÉ & ALERTES", R))
        if wd_alts:
            last = wd_alts[-1]
            t    = last.get("type", "?")
            col  = R if t == "RECUPERATION" else Y
            lines.append(
                f"  Watchdog P1+2  : {col}{BD}{t}{W} | "
                f"raison={last.get('raison','?')}"
            )
        else:
            lines.append(f"  Watchdog P1+2  : {G}✓ OK{W}")

        if wd_p3:
            last3 = wd_p3[-1]
            ta    = last3.get("type_alerte", last3.get("type", "?"))
            lines.append(
                f"  Watchdog P3    : {Y}{BD}{ta}{W}")
        else:
            lines.append(f"  Watchdog P3    : {G}✓ OK{W}")

        if emergencies:
            last_e = emergencies[-1]
            lines.append(
                f"  Urgences EDF   : {nb_ovr} override(s) | "
                f"dernier → {last_e.get('dept_urgent','?')}"
            )
        else:
            lines.append(f"  Urgences EDF   : {G}✓ Aucun override{W}")

        # ─── SECTION 5 : PERCEPTION P4 ────────────────────────────────────────
        lines.append(header("👁  PERCEPTION VISUELLE (Phase 4)", M))
        if room:
            conf  = room.get("confidence", 0)
            nom   = room.get("nom_salle", "?")
            mkr   = room.get("marker", "?")
            mtype = room.get("marker_type", "?")
            lines.append(
                f"  Salle détectée : {BD}{nom}{W} | "
                f"conf={G}{BD}{conf:.0%}{W} | "
                f"marker={mkr} ({mtype})")
        else:
            lines.append(f"  Salle détectée : {DIM}hors zone connue{W}")

        if human:
            count   = human.get("count", 0)
            dist_m  = human.get("dist_min_m", 0)
            humains = human.get("humains", [])
            col_h   = R if dist_m < 1.0 else (Y if dist_m < 2.5 else C)
            lines.append(
                f"  Humains proches: {col_h}{BD}{count}{W} personne(s) | "
                f"plus proche : {col_h}{dist_m:.1f}m{W}"
            )
            for h in humains[:2]:
                lines.append(
                    f"  └─ {h.get('role','?')} : {h.get('nom','?')} "
                    f"à {h.get('distance',0):.1f}m")
        else:
            lines.append(f"  Humains proches: {G}✓ Zone dégagée{W}")

        # ─── FOOTER ───────────────────────────────────────────────────────────
        lines.append(f"{BD}{C}{'─'*WIDTH}{W}")
        lines.append(
            f"{DIM}Topics actifs: {WIDTH//2 - 10} | "
            f"Phase 1+2+3+4 | "
            f"[Ctrl+C pour quitter]{W}\n")

        print("\n".join(lines))

    # ══════════════════════════════════════════════════════════════════════════
    # Publication résumé JSON
    # ══════════════════════════════════════════════════════════════════════════

    def _publier_summary(self):
        """Publie un résumé JSON complet pour intégration externe."""
        with self._lock:
            nb_ok  = self._nb_deadlines_ok
            nb_ko  = self._nb_deadlines_ko
            total  = nb_ok + nb_ko
            rate   = nb_ok / total * 100 if total else 0

            summary = {
                "timestamp": time.time(),
                "scheduler": {
                    "mission_active": self._tache_courante,
                    "deadlines_ok":   nb_ok,
                    "deadlines_ko":   nb_ko,
                    "success_rate_pct": round(rate, 1),
                    "edf_overrides":  self._nb_overrides,
                },
                "navigation": {
                    "state_p3":    self._nav_state_p3,
                    "etat_robot":  self._etat_robot.get("etat") if self._etat_robot else None,
                    "pos_x":       round(self._pos_x, 2),
                    "pos_y":       round(self._pos_y, 2),
                    "dept_cible":  self._dept_cible_nav,
                    "obstacles":   self._obstacle_count,
                    "metrics":     self._nav_metrics,
                },
                "livraisons": {
                    "total":    total,
                    "derniere": self._livraisons[-1] if self._livraisons else None,
                    "visuelles": len(self._delivery_confirmed_visual),
                },
                "securite": {
                    "watchdog_alerts":    len(self._watchdog_alertes),
                    "watchdog_alerts_p3": len(self._watchdog_alert_p3),
                    "emergency_events":   len(self._emergency_events),
                },
                "perception": {
                    "salle_courante": self._room_detected,
                    "human_detected": self._human_detected is not None,
                    "human_count":    self._human_detected.get("count", 0)
                                      if self._human_detected else 0,
                },
            }

        self._pub_summary.publish(String(data=json.dumps(summary)))


def main(args=None):
    rclpy.init(args=args)
    node = VisualDashboardP4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
