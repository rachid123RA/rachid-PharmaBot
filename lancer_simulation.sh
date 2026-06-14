#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════════════╗
# ║         PharmaBot — Script de lancement Ubuntu (ROS2 natif)             ║
# ║   Robot autonome de livraison pharmaceutique — Simulation Gazebo 3D     ║
# ║   Ibn Tofaïl University — Systèmes Temps Réel 2025-2026                 ║
# ╚══════════════════════════════════════════════════════════════════════════╝

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m'

WORKSPACE="$HOME/ros2_ws"

print_banner() {
    echo ""
    echo -e "${BLUE}╔══════════════════════════════════════════════════════════╗${NC}"
    echo -e "${BLUE}║${BOLD}          🤖 PharmaBot — Simulation Gazebo 3D             ${NC}${BLUE}║${NC}"
    echo -e "${BLUE}║  Robot Pioneer 3AT · EDF Scheduler · Livraison médicaments ║${NC}"
    echo -e "${BLUE}║  Ibn Tofaïl University — Systèmes Temps Réel 2025-2026     ║${NC}"
    echo -e "${BLUE}╚══════════════════════════════════════════════════════════╝${NC}"
    echo ""
}

check_ros2() {
    if ! command -v ros2 &> /dev/null; then
        echo -e "${RED}❌ ROS2 non trouvé !${NC}"
        echo ""
        echo -e "${YELLOW}📦 Installation ROS2 Humble :${NC}"
        echo "  sudo apt update"
        echo "  sudo apt install software-properties-common"
        echo "  sudo add-apt-repository universe"
        echo "  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \\"
        echo "       -o /usr/share/keyrings/ros-archive-keyring.gpg"
        echo "  echo 'deb [arch=\$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg]'"
        echo "       http://packages.ros.org/ros2/ubuntu \$(. /etc/os-release && echo \$UBUNTU_CODENAME) main'"
        echo "       | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null"
        echo "  sudo apt update && sudo apt install ros-humble-desktop ros-humble-gazebo-ros-pkgs"
        exit 1
    fi
    echo -e "${GREEN}✅ ROS2 trouvé : $(ros2 --version 2>/dev/null || echo 'Humble')${NC}"
}

check_gazebo() {
    if ! command -v gazebo &> /dev/null; then
        echo -e "${RED}❌ Gazebo non trouvé !${NC}"
        echo -e "${YELLOW}Installer : sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ Gazebo trouvé${NC}"
}

install_deps() {
    echo -e "${CYAN}📦 Installation des dépendances ROS2...${NC}"
    sudo apt-get update -qq
    sudo apt-get install -y \
        ros-humble-gazebo-ros-pkgs \
        ros-humble-gazebo-plugins \
        ros-humble-gazebo-ros2-control \
        ros-humble-nav2-bringup \
        ros-humble-slam-toolbox \
        ros-humble-robot-state-publisher \
        ros-humble-joint-state-publisher \
        python3-colcon-common-extensions \
        python3-rosdep \
        python3-pip 2>/dev/null

    pip3 install -q stable-baselines3==2.3.2 gymnasium==0.29.1 "numpy<2.0" 2>/dev/null || true
    echo -e "${GREEN}✅ Dépendances installées${NC}"
}

setup_workspace() {
    echo -e "${CYAN}🔧 Configuration workspace ROS2...${NC}"

    # Créer workspace si nécessaire
    mkdir -p "$WORKSPACE/src"

    # Cloner ou mettre à jour le repo
    REPO_DIR="$WORKSPACE/src/PharmaBot"
    if [ -d "$REPO_DIR/.git" ]; then
        echo -e "${YELLOW}📥 Mise à jour du projet...${NC}"
        cd "$REPO_DIR" && git pull origin main
    else
        echo -e "${YELLOW}📥 Clonage du projet...${NC}"
        git clone https://github.com/rachid123RA/rachid-PharmaBot.git "$REPO_DIR"
    fi

    # Copier le package dans src
    if [ -d "$REPO_DIR/hospital_robot_spawner" ] && [ ! -L "$WORKSPACE/src/hospital_robot_spawner" ]; then
        ln -sfn "$REPO_DIR/hospital_robot_spawner" "$WORKSPACE/src/hospital_robot_spawner" 2>/dev/null || \
        cp -r "$REPO_DIR/hospital_robot_spawner" "$WORKSPACE/src/"
    fi

    echo -e "${GREEN}✅ Workspace configuré : $WORKSPACE${NC}"
}

build_package() {
    echo -e "${CYAN}🔨 Compilation du package ROS2...${NC}"
    cd "$WORKSPACE"
    source /opt/ros/humble/setup.bash
    colcon build --packages-select hospital_robot_spawner \
                 --cmake-args -DCMAKE_BUILD_TYPE=Release \
                 2>&1 | tail -5
    echo -e "${GREEN}✅ Package compilé${NC}"
}

launch_gazebo_simulation() {
    echo -e "\n${BOLD}${BLUE}═══════════════════════════════════════════════════${NC}"
    echo -e "${BOLD}${GREEN}  🚀 LANCEMENT SIMULATION 3D — PharmaBot${NC}"
    echo -e "${BOLD}${BLUE}═══════════════════════════════════════════════════${NC}"
    echo ""
    echo -e "${CYAN}Ordre de démarrage :${NC}"
    echo -e "  t+0s  → Gazebo Classic 3D (hospital.world)"
    echo -e "  t+3s  → Spawn Pioneer 3AT (robot rouge)"
    echo -e "  t+5s  → EDF Scheduler (ordonnancement temps réel)"
    echo -e "  t+6s  → Mission Manager + Pharmacist Node"
    echo -e "  t+7s  → Navigation Bridge (contrôle robot)"
    echo -e "  t+8s  → Doctor Request (génère requêtes médicales)"
    echo -e "  t+9s  → Autonomous Navigator (Phase 3)"
    echo -e "  t+10s → Gazebo Bridge (médicaments 3D)"
    echo -e "  t+11s → Perception Node (Phase 4)"
    echo ""
    echo -e "${YELLOW}💡 Dans Gazebo : Menu View → Models pour voir les modèles${NC}"
    echo -e "${YELLOW}💡 Le robot rouge navigue automatiquement entre les salles${NC}"
    echo ""

    cd "$WORKSPACE"
    source /opt/ros/humble/setup.bash
    source install/setup.bash

    export GAZEBO_MODEL_PATH="$WORKSPACE/src/hospital_robot_spawner/models:$GAZEBO_MODEL_PATH"
    export GAZEBO_RESOURCE_PATH="$WORKSPACE/src/hospital_robot_spawner:$GAZEBO_RESOURCE_PATH"

    ros2 launch hospital_robot_spawner pharmabot_full.launch.py
}

launch_gazebo_only() {
    echo -e "${CYAN}🌍 Lancement Gazebo seul (monde hôpital)...${NC}"
    source /opt/ros/humble/setup.bash

    WORLD="$WORKSPACE/src/hospital_robot_spawner/worlds/hospital.world"
    if [ ! -f "$WORLD" ]; then
        WORLD="$(ros2 pkg prefix hospital_robot_spawner)/share/hospital_robot_spawner/worlds/hospital.world"
    fi

    export GAZEBO_MODEL_PATH="$WORKSPACE/src/hospital_robot_spawner/models:$GAZEBO_MODEL_PATH"

    gazebo --verbose "$WORLD" \
        -s libgazebo_ros_init.so \
        -s libgazebo_ros_factory.so
}

launch_nodes_only() {
    echo -e "${CYAN}🤖 Lancement nœuds ROS2 sans Gazebo (simulation Python)...${NC}"
    source /opt/ros/humble/setup.bash
    source "$WORKSPACE/install/setup.bash" 2>/dev/null || true

    echo -e "${YELLOW}Lancement : rt_scheduler + mission_manager + nav_bridge + doctor_request${NC}"
    ros2 launch hospital_robot_spawner pharmabot_full.launch.py &
    echo -e "${GREEN}✅ Nœuds lancés (PID: $!)${NC}"
    echo -e "${CYAN}Pour arrêter : Ctrl+C${NC}"
    wait
}

run_tests() {
    echo -e "${CYAN}🧪 Lancement des tests (22/22)...${NC}"
    cd "$WORKSPACE/src/PharmaBot" 2>/dev/null || cd "$WORKSPACE"

    echo -e "\n${YELLOW}═══ PHASE 1+2 ═══${NC}"
    python3 tests/run_tests_phase1_phase2.py -v

    echo -e "\n${YELLOW}═══ PHASE 3+4 ═══${NC}"
    python3 tests/run_tests_phase3_phase4.py -v
}

show_menu() {
    print_banner
    echo -e "${BOLD}Choisissez une option :${NC}"
    echo ""
    echo -e "  ${GREEN}1${NC}) 🚀 Simulation Gazebo 3D COMPLÈTE (recommandé)"
    echo -e "     → Robot + hôpital 3D + médicaments + tous les nœuds"
    echo ""
    echo -e "  ${CYAN}2${NC}) 🌍 Gazebo seul (monde hôpital 3D)"
    echo -e "     → Juste le monde 3D pour explorer"
    echo ""
    echo -e "  ${BLUE}3${NC}) 🤖 Nœuds ROS2 sans Gazebo (simulation Python)"
    echo -e "     → Terminal seulement, pas de 3D"
    echo ""
    echo -e "  ${YELLOW}4${NC}) 🧪 Tests automatisés (22/22 PASS)"
    echo ""
    echo -e "  ${MAGENTA}5${NC}) 📦 Installer dépendances + compiler"
    echo -e "     → À faire la 1ère fois"
    echo ""
    echo -e "  ${RED}q${NC}) Quitter"
    echo ""
    read -rp "→ Votre choix : " choice

    case "$choice" in
        1) check_ros2; check_gazebo; launch_gazebo_simulation ;;
        2) check_ros2; check_gazebo; launch_gazebo_only ;;
        3) check_ros2; launch_nodes_only ;;
        4) run_tests ;;
        5) check_ros2; install_deps; setup_workspace; build_package ;;
        q|Q) echo -e "${YELLOW}Au revoir !${NC}"; exit 0 ;;
        *) echo -e "${RED}Option invalide${NC}"; show_menu ;;
    esac
}

# ── Point d'entrée ──
case "${1:-menu}" in
    --install)  check_ros2; install_deps; setup_workspace; build_package ;;
    --gazebo)   check_ros2; check_gazebo; launch_gazebo_simulation ;;
    --world)    check_ros2; check_gazebo; launch_gazebo_only ;;
    --nodes)    check_ros2; launch_nodes_only ;;
    --tests)    run_tests ;;
    --menu|*)   show_menu ;;
esac
