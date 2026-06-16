#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
#  PharmaBot — Script de lancement VM Linux (Ubuntu 22.04 + ROS2 Humble)
#  Sans Docker — Exécuter directement dans la VM
#  Ibn Tofaïl University | 2025-2026 | Prof. Khaoula Boukir
# ═══════════════════════════════════════════════════════════════════════

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

BOLD="\033[1m"
RED="\033[31m"
GREEN="\033[32m"
YELLOW="\033[33m"
BLUE="\033[34m"
CYAN="\033[36m"
RESET="\033[0m"

clear
echo -e "${BOLD}${BLUE}"
echo "  ██████╗ ██╗  ██╗ █████╗ ██████╗ ███╗   ███╗ █████╗ ██████╗  ██████╗ ████████╗"
echo "  ██╔══██╗██║  ██║██╔══██╗██╔══██╗████╗ ████║██╔══██╗██╔══██╗██╔═══██╗╚══██╔══╝"
echo "  ██████╔╝███████║███████║██████╔╝██╔████╔██║███████║██████╔╝██║   ██║   ██║   "
echo "  ██╔═══╝ ██╔══██║██╔══██║██╔══██╗██║╚██╔╝██║██╔══██║██╔══██╗██║   ██║   ██║   "
echo "  ██║     ██║  ██║██║  ██║██║  ██║██║ ╚═╝ ██║██║  ██║██████╔╝╚██████╔╝   ██║   "
echo "  ╚═╝     ╚═╝  ╚═╝╚═╝  ╚═╝╚═╝  ╚═╝╚═╝     ╚═╝╚═╝  ╚═╝╚═════╝  ╚═════╝    ╚═╝"
echo -e "${RESET}"
echo -e "  ${BOLD}Robot Autonome de Livraison Pharmaceutique en Milieu Hospitalier${RESET}"
echo -e "  ${YELLOW}Ibn Tofaïl University — ROS2 Humble + Gazebo Classic + Nav2${RESET}"
echo ""
echo "═══════════════════════════════════════════════════════════════════════"
echo ""
echo -e "${BOLD}Options :${RESET}"
echo ""
echo -e "  ${GREEN}[1]${RESET}  🚀 Lancer simulation complète  (Gazebo + Nav2 + tous nœuds)"
echo -e "  ${GREEN}[2]${RESET}  💊 Envoyer requêtes médecin    (terminal séparé)"
echo -e "  ${GREEN}[3]${RESET}  🔨 Compiler le projet          (colcon build)"
echo -e "  ${GREEN}[4]${RESET}  🧪 Lancer tests Phase 1+2"
echo -e "  ${GREEN}[5]${RESET}  🧪 Lancer tests Phase 3+4"
echo -e "  ${CYAN}[6]${RESET}  📋 Voir topics actifs          (ros2 topic list)"
echo -e "  ${CYAN}[7]${RESET}  📊 Voir état robot             (ros2 topic echo)"
echo -e "  ${CYAN}[8]${RESET}  🗺️  Régénérer la carte hôpital"
echo -e "  ${CYAN}[9]${RESET}  ℹ️  État de l'installation"
echo -e "  ${RED}[0]${RESET}  ✖  Quitter"
echo ""
echo "═══════════════════════════════════════════════════════════════════════"
read -p "  Votre choix : " CHOIX
echo ""

# ── Sourcer ROS2 ──────────────────────────────────────────────────────
source_ros() {
    if [ -f /opt/ros/humble/setup.bash ]; then
        source /opt/ros/humble/setup.bash
    else
        echo -e "${RED}❌ ROS2 Humble non trouvé. Lancer INSTALL_VM.sh d'abord.${RESET}"
        exit 1
    fi
    INSTALL_DIR="$HOME/ros2_ws/install/setup.bash"
    if [ -f "$INSTALL_DIR" ]; then
        source "$INSTALL_DIR"
    else
        echo -e "${YELLOW}⚠  Workspace non compilé. Compilation en cours...${RESET}"
        compiler_projet
    fi
}

compiler_projet() {
    echo -e "${CYAN}▶ Compilation colcon...${RESET}"
    source /opt/ros/humble/setup.bash
    # Copier le package dans le workspace
    WS="$HOME/ros2_ws"
    mkdir -p "$WS/src"
    if [ ! -L "$WS/src/hospital_robot_spawner" ]; then
        ln -sf "$SCRIPT_DIR/hospital_robot_spawner" "$WS/src/hospital_robot_spawner"
        echo "   Lien symbolique créé → $WS/src/hospital_robot_spawner"
    fi
    cd "$WS"
    colcon build --symlink-install --packages-select hospital_robot_spawner 2>&1 | tail -20
    echo -e "${GREEN}✅ Compilation terminée${RESET}"
    source "$WS/install/setup.bash"
    cd "$SCRIPT_DIR"
}

case $CHOIX in

  1)
    echo -e "${BOLD}${GREEN}▶ Lancement simulation PharmaBot complète...${RESET}"
    echo ""
    echo -e "  ${CYAN}Ordre de démarrage :${RESET}"
    echo "    t+0s  → Gazebo Classic (hospital.world)"
    echo "    t+4s  → Map Server (carte hôpital)"
    echo "    t+5s  → Nav2 (AMCL + planner + controller)"
    echo "    t+8s  → EDF Scheduler + DMA + Pharmacist"
    echo "    t+9s  → Mission Manager + Watchdog"
    echo "    t+10s → Nav2 Navigator + Delivery Detector"
    echo "    t+12s → Doctor Request Node"
    echo "    t+14s → Dashboard temps réel"
    echo ""
    echo -e "  ${YELLOW}Appuyez sur Ctrl+C pour arrêter${RESET}"
    echo ""
    source_ros
    ros2 launch hospital_robot_spawner pharmabot_vm.launch.py
    ;;

  2)
    echo -e "${BOLD}${GREEN}▶ Simulateur de requêtes médecin${RESET}"
    echo ""
    source_ros
    python3 "$SCRIPT_DIR/send_requete_medecin.py"
    ;;

  3)
    echo -e "${BOLD}${CYAN}▶ Compilation du projet...${RESET}"
    compiler_projet
    ;;

  4)
    echo -e "${BOLD}${GREEN}▶ Tests Phase 1+2${RESET}"
    echo ""
    python3 tests/run_tests_phase1_phase2.py -v
    ;;

  5)
    echo -e "${BOLD}${GREEN}▶ Tests Phase 3+4${RESET}"
    echo ""
    python3 tests/run_tests_phase3_phase4.py -v
    ;;

  6)
    echo -e "${BOLD}${CYAN}▶ Topics ROS2 actifs${RESET}"
    source_ros
    echo ""
    ros2 topic list | sort
    ;;

  7)
    echo -e "${BOLD}${CYAN}▶ État robot en temps réel${RESET}"
    echo ""
    source_ros
    echo -e "${YELLOW}Topics disponibles :${RESET}"
    echo "  /pharmabot/etat_robot       → machine à états"
    echo "  /pharmabot/nav_status       → navigation"
    echo "  /pharmabot/livraison_confirmee → livraisons"
    echo "  /demo/odom                  → position robot"
    echo ""
    read -p "Topic à écouter [/pharmabot/etat_robot] : " TOPIC
    TOPIC="${TOPIC:-/pharmabot/etat_robot}"
    ros2 topic echo "$TOPIC"
    ;;

  8)
    echo -e "${BOLD}${CYAN}▶ Régénération de la carte...${RESET}"
    python3 hospital_robot_spawner/maps/generate_map.py
    echo ""
    echo -e "${GREEN}✅ Carte régénérée${RESET}"
    ;;

  9)
    echo -e "${BOLD}État de l'installation${RESET}"
    echo ""
    echo -n "  ROS2 Humble    : "
    [ -d /opt/ros/humble ] && echo -e "${GREEN}✓ installé${RESET}" || echo -e "${RED}✗ manquant${RESET}"
    echo -n "  Gazebo Classic : "
    command -v gazebo &>/dev/null && echo -e "${GREEN}✓ installé${RESET}" || echo -e "${RED}✗ manquant${RESET}"
    echo -n "  Nav2           : "
    [ -d /opt/ros/humble/lib/nav2_bringup ] && echo -e "${GREEN}✓ installé${RESET}" || echo -e "${RED}✗ manquant${RESET}"
    echo -n "  colcon         : "
    command -v colcon &>/dev/null && echo -e "${GREEN}✓ installé${RESET}" || echo -e "${RED}✗ manquant${RESET}"
    echo -n "  Python 3       : "
    python3 --version 2>&1
    echo ""
    echo -n "  Workspace build: "
    [ -f "$HOME/ros2_ws/install/setup.bash" ] && echo -e "${GREEN}✓ compilé${RESET}" || echo -e "${YELLOW}⚠ non compilé${RESET}"
    echo ""
    echo "  Fichiers projet :"
    [ -f hospital_robot_spawner/worlds/hospital.world ] && echo -e "    ${GREEN}✓${RESET} hospital.world" || echo -e "    ${RED}✗${RESET} hospital.world"
    [ -f hospital_robot_spawner/maps/hospital_map.yaml ] && echo -e "    ${GREEN}✓${RESET} hospital_map.yaml" || echo -e "    ${RED}✗${RESET} hospital_map.yaml"
    [ -f hospital_robot_spawner/maps/hospital_map.pgm ] && echo -e "    ${GREEN}✓${RESET} hospital_map.pgm" || echo -e "    ${RED}✗${RESET} hospital_map.pgm"
    [ -f hospital_robot_spawner/config/nav2_params.yaml ] && echo -e "    ${GREEN}✓${RESET} nav2_params.yaml" || echo -e "    ${RED}✗${RESET} nav2_params.yaml"
    echo ""
    ;;

  0)
    echo -e "${YELLOW}Au revoir !${RESET}"
    exit 0
    ;;

  *)
    echo -e "${RED}Choix invalide.${RESET}"
    ;;
esac

echo ""
echo "═══════════════════════════════════════════════════════════════════════"
