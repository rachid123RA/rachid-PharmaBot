#!/usr/bin/env python3
"""
pharmabot_gazebo_bridge.py — Pont Gazebo 3D pour PharmaBot

Ce nœud gère la simulation physique des médicaments dans Gazebo 3D :
  1. SPAWN       : Crée une boîte de médicaments dans la pharmacie
  2. PICKUP      : Quand le robot arrive en pharmacie → "attache" la boîte au robot
  3. TRANSPORT   : La boîte suit le robot pendant le trajet
  4. DELIVERY    : Quand le robot arrive au département → dépose la boîte sur la table

Utilise les services Gazebo :
  - /spawn_entity        (gazebo_msgs/SpawnEntity)
  - /delete_entity       (gazebo_msgs/DeleteEntity)
  - /get_entity_state    (gazebo_msgs/GetEntityState)
  - /set_entity_state    (gazebo_msgs/SetEntityState)

Souscrit à :
  - /pharmabot/tache_courante  → mission en cours
  - /demo/odom                 → position robot en temps réel
  - /pharmabot/nav_status      → phase navigation (RETOUR_PHARMACIE/NAVIGATION/ARRIVE)
  - /pharmabot/livraison_confirmee → livraison terminée

Publie sur :
  - /pharmabot/gazebo_status   → état de la simulation 3D
"""
import json
import math
import time
import threading
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion
from gazebo_msgs.srv import SpawnEntity, DeleteEntity, GetEntityState, SetEntityState
from gazebo_msgs.msg import EntityState

RESET  = "\033[0m"
BOLD   = "\033[1m"
CYAN   = "\033[96m"
GREEN  = "\033[92m"
RED    = "\033[91m"
YELLOW = "\033[93m"
BLUE   = "\033[94m"
MAGENTA= "\033[95m"

# Coordonnées des salles (identique à navigation_bridge.py)
SALLES = {
    "pharmacie":    {"x":  1.0, "y": 16.0},
    "reanimation":  {"x": 11.0, "y":  5.5},
    "urgences":     {"x": -5.0, "y": -6.6},
    "consultation": {"x": -2.0, "y":-27.0},
}

# Hauteur de la boîte sur le robot (au-dessus du dessus du châssis)
HAUTEUR_BOITE_SUR_ROBOT = 0.40  # mètres depuis le sol
# Hauteur de la boîte déposée sur la table de livraison
HAUTEUR_BOITE_TABLE = 0.97      # mètres (table=0.9 + moitié boîte)

# SDF de la boîte de médicaments (inline pour spawn)
MEDICATION_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
<model name="{name}">
  <link name="link">
    <inertial>
      <mass>0.3</mass>
      <inertia><ixx>0.001</ixx><ixy>0</ixy><ixz>0</ixz>
               <iyy>0.001</iyy><iyz>0</iyz><izz>0.001</izz></inertia>
    </inertial>
    <collision name="col">
      <geometry><box><size>0.20 0.12 0.10</size></box></geometry>
    </collision>
    <visual name="body">
      <geometry><box><size>0.20 0.12 0.10</size></box></geometry>
      <material>
        <ambient>{r} {g} {b} 1</ambient>
        <diffuse>{r} {g} {b} 1</diffuse>
      </material>
    </visual>
    <visual name="label">
      <pose>0 0 0.051 0 0 0</pose>
      <geometry><box><size>0.20 0.12 0.002</size></box></geometry>
      <material><ambient>0.1 0.7 0.2 1</ambient><diffuse>0.1 0.7 0.2 1</diffuse></material>
    </visual>
  </link>
</model>
</sdf>"""

# Couleurs des boîtes selon type de livraison
BOX_COLORS = {
    "HARD_RT": (1.0, 0.2, 0.2),   # Rouge — Réanimation
    "SOFT_RT": (1.0, 0.6, 0.0),   # Orange — Urgences
    "FIRM_RT": (0.2, 0.8, 0.2),   # Vert — Consultation
}


class PharmaGazeboBridge(Node):
    """
    Gère le cycle complet médicament → pharmacie → robot → département en 3D.
    """

    def __init__(self):
        super().__init__("pharmabot_gazebo_bridge")
        self.get_logger().info(
            f"{BOLD}{CYAN}╔═══════════════════════════════════════╗{RESET}")
        self.get_logger().info(
            f"{BOLD}{CYAN}║  PharmaBot Gazebo Bridge  — actif     ║{RESET}")
        self.get_logger().info(
            f"{BOLD}{CYAN}╚═══════════════════════════════════════╝{RESET}")

        # ── État interne ──
        self._robot_x       = SALLES["pharmacie"]["x"]
        self._robot_y       = SALLES["pharmacie"]["y"]
        self._robot_yaw     = 0.0
        self._mission       = None        # dict mission courante
        self._phase_nav     = "IDLE"      # phase navigation depuis nav_status
        self._dept_cible    = None
        self._type_rt       = "SOFT_RT"
        self._boite_active  = None        # nom de la boîte Gazebo en cours
        self._boite_etat    = "NONE"      # NONE / SUR_ETAGERE / EN_TRANSIT / LIVREE
        self._boite_counter = 0           # compteur pour noms uniques
        self._lock          = threading.Lock()
        self._transport_actif = False

        # ── Clients Gazebo ──
        self._cli_spawn  = self.create_client(SpawnEntity,   "/spawn_entity")
        self._cli_delete = self.create_client(DeleteEntity,  "/delete_entity")
        self._cli_set    = self.create_client(SetEntityState,"/gazebo/set_entity_state")

        # ── Publisher status ──
        self._pub_status = self.create_publisher(String, "/pharmabot/gazebo_status", 10)

        # ── Subscribers ──
        self.create_subscription(String,  "/pharmabot/tache_courante",    self._cb_mission,      10)
        self.create_subscription(Odometry,"/demo/odom",                   self._cb_odom,          1)
        self.create_subscription(String,  "/pharmabot/nav_status",        self._cb_nav_status,   10)
        self.create_subscription(String,  "/pharmabot/livraison_confirmee",self._cb_livraison,   10)

        # ── Timer transport (20Hz) ──
        self._timer_transport = self.create_timer(0.05, self._update_transport)

        self.get_logger().info(f"{GREEN}En attente des missions EDF...{RESET}")

    # ──────────────────────────────────────────────────────────────────────
    # CALLBACKS
    # ──────────────────────────────────────────────────────────────────────

    def _cb_mission(self, msg: String):
        """Nouvelle mission EDF reçue → préparer la boîte en pharmacie."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        dept    = data.get("departement", "")
        type_rt = data.get("type_rt", "SOFT_RT")
        med     = data.get("medicament", "Médicament standard")

        if dept == "pharmacie" or not dept:
            return

        with self._lock:
            old_dept = self._dept_cible
            self._mission  = data
            self._dept_cible = dept
            self._type_rt  = type_rt

        if dept != old_dept:
            self.get_logger().info(
                f"{BOLD}{YELLOW}▶ Nouvelle mission : {dept} ({type_rt}){RESET}")
            self.get_logger().info(
                f"  Médicament : {BOLD}{med}{RESET}")
            # Spawner la boîte en pharmacie (sur l'étagère)
            self._spawn_boite_pharmacie(dept, type_rt, med)

    def _cb_odom(self, msg: Odometry):
        """Mise à jour position robot en temps réel."""
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y
        # Calculer yaw depuis quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._robot_yaw = math.atan2(siny, cosy)

    def _cb_nav_status(self, msg: String):
        """Phase navigation changée."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return
        phase = data.get("phase", data.get("etat", ""))
        if not phase:
            return

        old_phase = self._phase_nav
        self._phase_nav = phase

        if phase == "RETOUR_PHARMACIE" and old_phase != "RETOUR_PHARMACIE":
            self.get_logger().info(f"{CYAN}→ Robot en route vers la pharmacie...{RESET}")

        elif phase == "CHARGEMENT" and old_phase != "CHARGEMENT":
            self.get_logger().info(
                f"{BOLD}{GREEN}🔄 Chargement médicaments en cours...{RESET}")
            # Démarrer le transport dans 1 seconde
            self._start_transport_delayed(1.0)

        elif phase == "NAVIGATION" and old_phase not in ("NAVIGATION",):
            dept = self._dept_cible
            if dept:
                self.get_logger().info(
                    f"{BOLD}{BLUE}🚀 Robot en route vers {dept}...{RESET}")
            self._transport_actif = True

        elif phase in ("ARRIVE", "GOAL_REACHED") and old_phase not in ("ARRIVE","GOAL_REACHED"):
            self.get_logger().info(
                f"{BOLD}{GREEN}✅ Robot arrivé à destination !{RESET}")

    def _cb_livraison(self, msg: String):
        """Livraison confirmée → déposer la boîte sur la table."""
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        dept = data.get("departement", self._dept_cible)
        if not dept or dept == "pharmacie":
            return

        self.get_logger().info(
            f"{BOLD}{GREEN}📦 Livraison confirmée → {dept}{RESET}")
        self._transport_actif = False

        # Déposer la boîte sur la table de livraison
        if self._boite_active and self._boite_etat == "EN_TRANSIT":
            self._deposer_boite(dept)

    # ──────────────────────────────────────────────────────────────────────
    # ACTIONS GAZEBO
    # ──────────────────────────────────────────────────────────────────────

    def _spawn_boite_pharmacie(self, dept: str, type_rt: str, med: str):
        """Spawn une boîte colorée sur l'étagère de la pharmacie."""
        if not self._cli_spawn.service_is_ready():
            self.get_logger().warn("Service /spawn_entity non disponible")
            return

        # Supprimer l'ancienne boîte si elle existe
        if self._boite_active:
            self._delete_entity(self._boite_active)

        with self._lock:
            self._boite_counter += 1
            name = f"med_delivery_{self._boite_counter}"
            self._boite_active = name
            self._boite_etat   = "SUR_ETAGERE"

        # Choisir couleur selon priorité
        r, g, b = BOX_COLORS.get(type_rt, (0.9, 0.9, 0.9))

        sdf = MEDICATION_SDF.format(name=name, r=r, g=g, b=b)

        request = SpawnEntity.Request()
        request.name        = name
        request.xml         = sdf
        request.robot_namespace = ""
        # Position : sur l'étagère de la pharmacie, légèrement soulevée
        request.initial_pose.position.x = SALLES["pharmacie"]["x"] - 0.5
        request.initial_pose.position.y = SALLES["pharmacie"]["y"] + 3.0
        request.initial_pose.position.z = 1.15   # sur la planche 2 de l'étagère
        request.initial_pose.orientation.w = 1.0

        future = self._cli_spawn.call_async(request)
        future.add_done_callback(
            lambda f: self.get_logger().info(
                f"{GREEN}📦 Boîte [{name}] ({type_rt}) spawné en pharmacie{RESET}"))

    def _delete_entity(self, name: str):
        """Supprimer une entité Gazebo."""
        if not self._cli_delete.service_is_ready():
            return
        req = DeleteEntity.Request()
        req.name = name
        self._cli_delete.call_async(req)

    def _deposer_boite(self, dept: str):
        """Déposer la boîte sur la table de livraison du département."""
        if not self._cli_set.service_is_ready():
            self.get_logger().warn("Service set_entity_state non disponible")
            return

        pos = SALLES.get(dept)
        if not pos:
            return

        req = SetEntityState.Request()
        req.state.name = self._boite_active
        req.state.reference_frame = "world"
        req.state.pose.position.x = pos["x"]
        req.state.pose.position.y = pos["y"]
        req.state.pose.position.z = HAUTEUR_BOITE_TABLE
        req.state.pose.orientation.w = 1.0

        future = self._cli_set.call_async(req)
        future.add_done_callback(
            lambda f: self._on_boite_deposee(dept))

    def _on_boite_deposee(self, dept: str):
        with self._lock:
            self._boite_etat = "LIVREE"
        self.get_logger().info(
            f"{BOLD}{GREEN}✅ Médicament déposé sur la table — {dept}{RESET}")
        self._pub_status_msg(f"LIVREE:{dept}:{self._boite_active}")

    def _start_transport_delayed(self, delay: float):
        """Démarrer le transport après un délai (simulation chargement)."""
        def _start():
            time.sleep(delay)
            with self._lock:
                if self._boite_active:
                    self._boite_etat = "EN_TRANSIT"
                    self._transport_actif = True
            self.get_logger().info(
                f"{BOLD}{CYAN}📦 Médicament chargé sur le robot → départ{RESET}")
        threading.Thread(target=_start, daemon=True).start()

    def _update_transport(self):
        """Mettre à jour la position de la boîte pour qu'elle suive le robot."""
        if not self._transport_actif:
            return
        with self._lock:
            etat = self._boite_etat
            name = self._boite_active
            rx   = self._robot_x
            ry   = self._robot_y
            yaw  = self._robot_yaw

        if etat != "EN_TRANSIT" or not name:
            return
        if not self._cli_set.service_is_ready():
            return

        # Position : légèrement devant le robot, en hauteur
        offset_x = 0.1 * math.cos(yaw)
        offset_y = 0.1 * math.sin(yaw)

        req = SetEntityState.Request()
        req.state.name            = name
        req.state.reference_frame = "world"
        req.state.pose.position.x = rx + offset_x
        req.state.pose.position.y = ry + offset_y
        req.state.pose.position.z = HAUTEUR_BOITE_SUR_ROBOT
        req.state.pose.orientation.w = 1.0

        self._cli_set.call_async(req)

    def _pub_status_msg(self, status: str):
        msg = String()
        msg.data = json.dumps({
            "status":       status,
            "boite":        self._boite_active,
            "etat":         self._boite_etat,
            "dept_cible":   self._dept_cible,
            "timestamp":    time.time(),
        })
        self._pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PharmaGazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
