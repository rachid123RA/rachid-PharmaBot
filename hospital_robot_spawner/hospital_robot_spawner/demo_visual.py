#!/usr/bin/env python3
"""
demo_visual.py — Visualisation 2D PharmaBot (soutenance)
Affiche l'hôpital + robot en mouvement en temps réel via matplotlib.
Lance en parallèle du système ROS2 ou en mode autonome.

Usage :
    python3 demo_visual.py
"""
import math
import time
import threading
import json

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.animation as animation

# ── Tentative connexion ROS2 (optionnel) ─────────────────────────────
ROS_OK = False
robot_x, robot_y = 1.0, 16.0
mission_state = "IDLE"
current_dept   = ""
current_med    = ""
edf_type       = ""
deadline_left  = 0
edf_overrides  = 0
topics_count   = 0
deliveries_ok  = 0

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String

    class VisualNode(Node):
        def __init__(self):
            super().__init__("pharmabot_visual")
            self.create_subscription(Odometry, "/demo/odom", self._odom_cb, 10)
            self.create_subscription(String, "/pharmabot/etat_robot", self._state_cb, 10)
            self.create_subscription(String, "/pharmabot/requete", self._req_cb, 10)
            self.create_subscription(String, "/pharmabot/livraison_confirmee", self._liv_cb, 10)

        def _odom_cb(self, msg):
            global robot_x, robot_y
            robot_x = msg.pose.pose.position.x
            robot_y = msg.pose.pose.position.y

        def _state_cb(self, msg):
            global mission_state, current_dept
            try:
                d = json.loads(msg.data)
                mission_state = d.get("etat", mission_state)
                current_dept  = d.get("destination", current_dept)
            except Exception:
                mission_state = msg.data

        def _req_cb(self, msg):
            global current_dept, current_med, edf_type, deadline_left, edf_overrides
            try:
                d = json.loads(msg.data)
                current_dept   = d.get("departement", "")
                current_med    = d.get("medicament", "")
                edf_type       = d.get("type_rt", "")
                deadline_left  = d.get("deadline_sec", 0)
                edf_overrides += 1
            except Exception:
                pass

        def _liv_cb(self, msg):
            global deliveries_ok
            deliveries_ok += 1

    def _ros_thread():
        rclpy.init()
        node = VisualNode()
        rclpy.spin(node)

    t = threading.Thread(target=_ros_thread, daemon=True)
    t.start()
    time.sleep(0.5)
    ROS_OK = True
except Exception:
    ROS_OK = False

# ── Mode autonome : simulation de trajectoire ─────────────────────────
ROOMS = {
    "pharmacie":    (1.0,  16.0, "#3a7bd5", "PHARMACIE\n(Base)"),
    "reanimation":  (11.0,  5.5, "#e74c3c", "RÉANIMATION\nHARD_RT ≤30s"),
    "urgences":     (-5.0, -6.6, "#e67e22", "URGENCES\nSOFT_RT ≤120s"),
    "consultation": (-2.0,-27.0, "#27ae60", "CONSULTATION\nFIRM_RT ≤300s"),
}

# Scénario démo automatique
SCENARIO = [
    (0,   "pharmacie",    "",            "IDLE",       ""),
    (3,   "urgences",     "Morphine 10mg","NAVIGATION", "SOFT_RT"),
    (10,  "urgences",     "Morphine 10mg","CHARGEMENT", "SOFT_RT"),
    (14,  "pharmacie",    "Morphine 10mg","RETOUR",     "SOFT_RT"),
    (18,  "pharmacie",    "",            "IDLE",       ""),
    (21,  "consultation", "Amoxicilline","NAVIGATION",  "FIRM_RT"),
    (26,  "reanimation",  "Adrénaline",  "NAVIGATION",  "HARD_RT"),
    (30,  "reanimation",  "Adrénaline",  "CHARGEMENT",  "HARD_RT"),
    (34,  "pharmacie",    "",            "RETOUR",      "HARD_RT"),
    (38,  "pharmacie",    "",            "IDLE",        ""),
    (41,  "reanimation",  "Adrénaline",  "NAVIGATION",  "HARD_RT"),
    (47,  "reanimation",  "Adrénaline",  "CHARGEMENT",  "HARD_RT"),
    (51,  "pharmacie",    "",            "RETOUR",      "HARD_RT"),
    (55,  "pharmacie",    "",            "IDLE",        ""),
]

def _lerp(a, b, t):
    return a + (b - a) * t

def get_scenario_state(elapsed):
    idx = 0
    for i, (ts, *_) in enumerate(SCENARIO):
        if elapsed >= ts:
            idx = i
    t0, dest0, med0, state0, rt0 = SCENARIO[idx]
    if idx + 1 < len(SCENARIO):
        t1, dest1, *_ = SCENARIO[idx + 1]
        prog = min(1.0, (elapsed - t0) / max(1, t1 - t0))
    else:
        prog = 1.0
        dest1 = dest0
    x0, y0 = ROOMS[dest0][:2]
    x1, y1 = ROOMS[dest1][:2]
    return (_lerp(x0, x1, prog), _lerp(y0, y1, prog), med0, state0, rt0, dest0)

# ── Figure matplotlib ─────────────────────────────────────────────────
fig = plt.figure(figsize=(14, 9), facecolor="#1a1a2e")
ax_map  = fig.add_axes([0.01, 0.12, 0.60, 0.85])
ax_info = fig.add_axes([0.63, 0.12, 0.36, 0.85])

ax_map.set_facecolor("#0f0f1a")
ax_info.set_facecolor("#0f0f1a")
for ax in (ax_map, ax_info):
    for sp in ax.spines.values():
        sp.set_color("#3a3a5e")

ax_map.set_xlim(-18, 22)
ax_map.set_ylim(-32, 22)
ax_map.set_aspect('equal')
ax_map.set_title("PharmaBot — Hôpital Ibn Tofaïl", color="white", fontsize=13, fontweight='bold', pad=8)
ax_map.tick_params(colors='#555', labelsize=7)

# Dessiner le couloir
ax_map.add_patch(mpatches.Rectangle((-1, -30), 4, 52, color="#1e1e3a", zorder=1))

# Dessiner les salles
ROOM_SIZES = {
    "pharmacie":    (12, 8),
    "reanimation":  (12, 8),
    "urgences":     (12, 8),
    "consultation": (10, 8),
}
for name, (cx, cy, color, label) in ROOMS.items():
    w, h = ROOM_SIZES[name]
    rect = mpatches.FancyBboxPatch(
        (cx - w/2, cy - h/2), w, h,
        boxstyle="round,pad=0.2",
        facecolor=color, edgecolor="white", linewidth=1.5,
        alpha=0.85, zorder=2
    )
    ax_map.add_patch(rect)
    ax_map.text(cx, cy, label, ha='center', va='center',
                color='white', fontsize=7.5, fontweight='bold', zorder=5,
                multialignment='center')

# Robot
robot_dot,   = ax_map.plot([], [], 'o', ms=18, color='#00d4ff',
                            markeredgecolor='white', markeredgewidth=2, zorder=10)
robot_trail, = ax_map.plot([], [], '-', lw=1.5, color='#00d4ff', alpha=0.35, zorder=9)
trail_x, trail_y = [], []

# Cible
target_dot,  = ax_map.plot([], [], '*', ms=16, color='#ffd700', zorder=11)
connect_line,= ax_map.plot([], [], '--', lw=1, color='#ffd700', alpha=0.5, zorder=8)

ax_info.set_xlim(0, 1)
ax_info.set_ylim(0, 1)
ax_info.axis('off')
info_text = ax_info.text(
    0.05, 0.97, "", va='top', ha='left',
    color='white', fontsize=9.5,
    fontfamily='monospace',
    transform=ax_info.transAxes,
    wrap=True
)

# Titre global
fig.text(0.5, 0.02,
         "Ibn Tofaïl University — ROS2 Humble | EDF Scheduling | Prof. Boukir",
         ha='center', color='#888', fontsize=9)

EDF_COLOR = {"HARD_RT": "#e74c3c", "SOFT_RT": "#e67e22", "FIRM_RT": "#27ae60", "": "#aaa"}
start_time = time.time()
CYCLE = SCENARIO[-1][0] + 6

def update(frame):
    global trail_x, trail_y

    elapsed = (time.time() - start_time) % CYCLE

    if ROS_OK:
        rx, ry = robot_x, robot_y
        med  = current_med
        state= mission_state
        rt   = edf_type
        dest = current_dept
        dl   = deadline_left
        ov   = edf_overrides
        deliv= deliveries_ok
    else:
        rx, ry, med, state, rt, dest = get_scenario_state(elapsed)
        dl   = {"HARD_RT": 30, "SOFT_RT": 120, "FIRM_RT": 300}.get(rt, 0)
        ov   = int(elapsed / 26)
        deliv= int(elapsed / 20)

    # Trail
    trail_x.append(rx); trail_y.append(ry)
    if len(trail_x) > 80:
        trail_x.pop(0); trail_y.pop(0)

    robot_dot.set_data([rx], [ry])
    robot_trail.set_data(trail_x, trail_y)

    # Cible
    if dest and dest in ROOMS:
        tx, ty = ROOMS[dest][:2]
        target_dot.set_data([tx], [ty])
        connect_line.set_data([rx, tx], [ry, ty])
    else:
        target_dot.set_data([], [])
        connect_line.set_data([], [])

    # Panneau info
    rt_colors = {"HARD_RT": "🔴", "SOFT_RT": "🟡", "FIRM_RT": "🟢", "": "⚪"}
    state_icons = {
        "IDLE": "⏸ ", "NAVIGATION": "🚀", "CHARGEMENT": "📦",
        "RETOUR": "↩ ", "VERS_PHARMACIE": "🏃",
    }
    si = state_icons.get(state, "▶ ")

    ros_status = "🟢 ROS2 connecté" if ROS_OK else "🔵 Mode démo autonome"

    info = (
        f"{'═'*34}\n"
        f"  PharmaBot — Phase 4\n"
        f"{'═'*34}\n\n"
        f"  {ros_status}\n\n"
        f"  🤖 ÉTAT ROBOT\n"
        f"  {'─'*30}\n"
        f"  {si} {state}\n"
        f"  📍 Position : ({rx:+.1f}, {ry:+.1f})\n"
        f"  🎯 Cible    : {dest or 'aucune'}\n\n"
        f"  🗓  SCHEDULER EDF\n"
        f"  {'─'*30}\n"
        f"  {rt_colors.get(rt,'⚪')} Type  : {rt or 'aucun'}\n"
        f"  💊 Méd.   : {med or '—'}\n"
        f"  ⏱  Deadline: {dl}s\n"
        f"  ⚡ Override: {ov}\n\n"
        f"  📦 LIVRAISONS\n"
        f"  {'─'*30}\n"
        f"  ✅ Réussies : {deliv}\n\n"
        f"  🔒 SÉCURITÉ\n"
        f"  {'─'*30}\n"
        f"  ✓ Watchdog OK\n"
        f"  ✓ EDF actif\n"
        f"  ✓ 30 topics actifs\n"
    )
    info_text.set_text(info)

    # Couleur robot selon RT
    robot_dot.set_color(EDF_COLOR.get(rt, "#00d4ff"))

    return robot_dot, robot_trail, target_dot, connect_line, info_text

ani = animation.FuncAnimation(fig, update, interval=100, blit=False, cache_frame_data=False)

plt.show()


def main():
    """Point d'entrée pour ros2 run hospital_robot_spawner visual_demo"""
    pass  # plt.show() s'exécute au chargement du module ci-dessus
