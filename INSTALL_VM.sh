#!/bin/bash
# ═══════════════════════════════════════════════════════════════════════
#  INSTALL_VM.sh — Installation complète PharmaBot de A à Z
#  Ubuntu 22.04 LTS (VM UTM/VMware sur Mac ou VM VirtualBox)
#  ROS2 Humble + Gazebo Classic 11 + Nav2 + Python deps
#  Ibn Tofaïl University | 2025-2026
#
#  UTILISATION :
#     chmod +x INSTALL_VM.sh
#     ./INSTALL_VM.sh
# ═══════════════════════════════════════════════════════════════════════

set -e

BOLD="\033[1m"
GREEN="\033[32m"
CYAN="\033[36m"
YELLOW="\033[33m"
RED="\033[31m"
RESET="\033[0m"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log()  { echo -e "${CYAN}▶ $1${RESET}"; }
ok()   { echo -e "${GREEN}✅ $1${RESET}"; }
warn() { echo -e "${YELLOW}⚠  $1${RESET}"; }
err()  { echo -e "${RED}❌ $1${RESET}"; }

echo -e "${BOLD}${CYAN}"
echo "╔══════════════════════════════════════════════════════════════╗"
echo "║   PharmaBot — Installation complète VM Ubuntu 22.04        ║"
echo "║   ROS2 Humble + Gazebo Classic + Nav2 + Python             ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo -e "${RESET}"
echo ""

# ── Vérification Ubuntu 22.04 ─────────────────────────────────────────
if ! grep -q "22.04" /etc/os-release 2>/dev/null; then
    warn "Ce script est prévu pour Ubuntu 22.04. Continuer quand même ? (o/n)"
    read -r rep
    [[ "$rep" != "o" ]] && exit 1
fi

# ════════════════════════════════════════════════════════════════════
# NETTOYAGE PRÉVENTIF — Supprimer toute entrée OSRF conflictuelle
# (doit être fait AVANT le premier apt-get update)
# ════════════════════════════════════════════════════════════════════
log "Nettoyage des dépôts OSRF conflictuels..."
sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list \
           /etc/apt/sources.list.d/gazebo*.list
sudo rm -f /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
           /usr/share/keyrings/gazebo-archive-keyring.gpg
ok "Dépôts OSRF nettoyés"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 1 — Mise à jour système
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 1 — Mise à jour système"
sudo apt-get update -qq
sudo apt-get upgrade -y -qq
ok "Système à jour"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 2 — Locale UTF-8
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 2 — Configuration locale"
sudo apt-get install -y -qq locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
ok "Locale configurée"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 3 — Dépendances système
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 3 — Dépendances système"
sudo apt-get install -y -qq \
    curl wget git python3-pip python3-dev \
    build-essential cmake gnupg2 lsb-release \
    software-properties-common apt-transport-https \
    python3-colcon-common-extensions \
    python3-rosdep python3-vcstool \
    libeigen3-dev libboost-all-dev
ok "Dépendances système installées"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 4 — ROS2 Humble
# ════════════════════════════════════════════════════════════════════
if [ ! -d /opt/ros/humble ]; then
    log "ÉTAPE 4 — Installation ROS2 Humble"

    # Clé GPG
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg

    # Repository
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

    sudo apt-get update -qq
    sudo apt-get install -y ros-humble-desktop
    ok "ROS2 Humble installé"
else
    ok "ROS2 Humble déjà installé"
fi

# Source ROS2
source /opt/ros/humble/setup.bash

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 5 — Gazebo Classic 11
# ════════════════════════════════════════════════════════════════════
if ! command -v gazebo &>/dev/null; then
    log "ÉTAPE 5 — Ajout dépôt OSRF + Gazebo Classic 11"

    # Activer universe (nécessaire pour certaines dépendances Gazebo)
    sudo add-apt-repository universe -y

    # Supprimer toute entrée OSRF conflictuelle existante
    sudo rm -f /etc/apt/sources.list.d/gazebo-stable.list \
               /etc/apt/sources.list.d/gazebo*.list \
               /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
               /usr/share/keyrings/gazebo-archive-keyring.gpg

    # Ajouter le dépôt officiel OSRF (Gazebo) — clé en format binaire (dearmor requis)
    wget -qO- https://packages.osrfoundation.org/gazebo.key \
        | sudo gpg --dearmor -o /usr/share/keyrings/gazebo-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

    sudo apt-get update -qq

    # Gazebo Classic 11
    sudo apt-get install -y gazebo libgazebo11-dev

    # Packages ROS2 ↔ Gazebo (inclut ros-humble-gazebo-ros et plugins)
    sudo apt-get install -y ros-humble-gazebo-ros-pkgs

    ok "Gazebo Classic 11 installé"
else
    ok "Gazebo déjà installé : $(gazebo --version 2>&1 | head -1)"
fi

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 6 — Nav2
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 6 — Navigation2 (Nav2)"
sudo apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-msgs \
    ros-humble-nav2-map-server \
    ros-humble-nav2-amcl \
    ros-humble-nav2-lifecycle-manager \
    ros-humble-nav2-planner \
    ros-humble-nav2-controller \
    ros-humble-nav2-bt-navigator \
    ros-humble-nav2-behaviors \
    ros-humble-nav2-navfn-planner \
    ros-humble-dwb-core \
    ros-humble-dwb-critics \
    ros-humble-dwb-plugins \
    ros-humble-nav2-costmap-2d \
    ros-humble-nav2-smoother
ok "Nav2 installé"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 7 — Packages ROS2 supplémentaires
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 7 — Packages ROS2 supplémentaires"
sudo apt-get install -y \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-nav-msgs \
    ros-humble-action-msgs \
    ros-humble-rclpy \
    ros-humble-ament-cmake \
    ros-humble-ament-python \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher
ok "Packages ROS2 installés"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 8 — Python packages
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 8 — Packages Python"
pip3 install --upgrade --quiet \
    "numpy<2.0" \
    "stable-baselines3==2.3.2" \
    "gymnasium==0.29.1" \
    "optuna==3.6.1" \
    tensorboard \
    rich
ok "Packages Python installés"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 9 — Workspace ROS2
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 9 — Création workspace ROS2"
WS="$HOME/ros2_ws"
mkdir -p "$WS/src"

# Lien symbolique vers le projet
if [ ! -L "$WS/src/hospital_robot_spawner" ]; then
    ln -sf "$SCRIPT_DIR/hospital_robot_spawner" "$WS/src/hospital_robot_spawner"
    ok "Lien symbolique créé → $WS/src/hospital_robot_spawner"
else
    ok "Lien symbolique déjà existant"
fi

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 10 — Génération de la carte hôpital
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 10 — Génération carte hôpital"
python3 "$SCRIPT_DIR/hospital_robot_spawner/maps/generate_map.py"
ok "Carte générée"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 11 — Compilation colcon
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 11 — Compilation colcon"
cd "$WS"
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select hospital_robot_spawner
ok "Compilation réussie"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 12 — Fichier ~/.bashrc
# ════════════════════════════════════════════════════════════════════
log "ÉTAPE 12 — Configuration .bashrc"
BASHRC="$HOME/.bashrc"

add_if_missing() {
    grep -qF "$1" "$BASHRC" || echo "$1" >> "$BASHRC"
}

add_if_missing "source /opt/ros/humble/setup.bash"
add_if_missing "source $WS/install/setup.bash"
add_if_missing "export GAZEBO_MODEL_PATH=$HOME/ros2_ws/install/hospital_robot_spawner/share/hospital_robot_spawner/models:\$GAZEBO_MODEL_PATH"
add_if_missing "export ROS_DOMAIN_ID=42"
add_if_missing "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp"

source "$BASHRC" 2>/dev/null || true
ok ".bashrc configuré"

# ════════════════════════════════════════════════════════════════════
# ÉTAPE 13 — Rendre les scripts exécutables
# ════════════════════════════════════════════════════════════════════
chmod +x "$SCRIPT_DIR/lancer_vm.sh"
chmod +x "$SCRIPT_DIR/send_requete_medecin.py"
ok "Scripts exécutables"

# ════════════════════════════════════════════════════════════════════
# FIN — Résumé
# ════════════════════════════════════════════════════════════════════
cd "$SCRIPT_DIR"
echo ""
echo -e "${BOLD}${GREEN}╔══════════════════════════════════════════════════════════════╗${RESET}"
echo -e "${BOLD}${GREEN}║         ✅  INSTALLATION TERMINÉE AVEC SUCCÈS !             ║${RESET}"
echo -e "${BOLD}${GREEN}╚══════════════════════════════════════════════════════════════╝${RESET}"
echo ""
echo -e "${BOLD}Pour lancer PharmaBot :${RESET}"
echo ""
echo -e "  ${CYAN}cd $SCRIPT_DIR${RESET}"
echo -e "  ${CYAN}./lancer_vm.sh${RESET}  → puis choisir ${GREEN}[1]${RESET}"
echo ""
echo -e "${BOLD}Dans un 2e terminal (requêtes médecin) :${RESET}"
echo ""
echo -e "  ${CYAN}cd $SCRIPT_DIR${RESET}"
echo -e "  ${CYAN}./lancer_vm.sh${RESET}  → puis choisir ${GREEN}[2]${RESET}"
echo ""
echo -e "${YELLOW}⚠  Ouvrir un nouveau terminal pour que .bashrc soit rechargé${RESET}"
echo ""
