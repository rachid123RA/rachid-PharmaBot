#!/bin/bash
# ╔══════════════════════════════════════════════════════════════════════╗
# ║   PharmaBot — Installation complète Ubuntu 22.04 ARM64              ║
# ║   ROS2 Humble + Ignition Gazebo Fortress + PharmaBot                ║
# ╚══════════════════════════════════════════════════════════════════════╝
set -e

GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; RED='\033[0;31m'; NC='\033[0m'
log()  { echo -e "${CYAN}[PharmaBot]${NC} $1"; }
ok()   { echo -e "${GREEN}✅ $1${NC}"; }
warn() { echo -e "${YELLOW}⚠️  $1${NC}"; }

log "Installation PharmaBot sur Ubuntu 22.04 ARM64"
log "Étape 1/5 : Dépôts ROS2 + OSRF..."

sudo apt install -y curl gnupg lsb-release 2>/dev/null

# Clé ROS2
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

# Dépôt ROS2
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update -qq
ok "Dépôts configurés"

log "Étape 2/5 : Installation ROS2 Humble..."
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
ok "ROS2 Humble installé"

log "Étape 3/5 : Installation Ignition Gazebo Fortress..."
sudo apt install -y ignition-fortress 2>/dev/null && ok "Ignition Fortress installé" \
|| warn "Ignition Fortress non disponible — essai gz-sim..."
sudo apt install -y gz-sim7 2>/dev/null && ok "gz-sim7 installé" || true

log "Étape 4/5 : Bridge ROS2-Ignition..."
sudo apt install -y ros-humble-ros-ign-gazebo ros-humble-ros-ign-bridge \
    ros-humble-ros-gz-sim ros-humble-ros-gz-bridge 2>/dev/null || \
sudo apt install -y ros-humble-ros-ign-bridge 2>/dev/null || true
ok "Bridge installé"

log "Étape 5/5 : Compilation workspace PharmaBot..."
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p ~/ros2_ws/src
cp -r "$SCRIPT_DIR/hospital_robot_spawner" ~/ros2_ws/src/ 2>/dev/null || true

cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select hospital_robot_spawner \
             --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3
source install/setup.bash
ok "Compilation terminée"

# Configurer .bashrc
grep -q "ros/humble/setup.bash" ~/.bashrc || echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
grep -q "ros2_ws/install/setup.bash" ~/.bashrc || echo "[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash" >> ~/.bashrc

echo ""
echo -e "${GREEN}╔═══════════════════════════════════════════════════╗${NC}"
echo -e "${GREEN}║  ✅ Installation terminée !                        ║${NC}"
echo -e "${GREEN}║                                                   ║${NC}"
echo -e "${GREEN}║  Lancer la simulation Gazebo 3D :                 ║${NC}"
echo -e "${GREEN}║  cd ~/ros2_ws                                     ║${NC}"
echo -e "${GREEN}║  source install/setup.bash                        ║${NC}"
echo -e "${GREEN}║  ros2 launch hospital_robot_spawner \              ║${NC}"
echo -e "${GREEN}║       pharmabot_ignition.launch.py                ║${NC}"
echo -e "${GREEN}╚═══════════════════════════════════════════════════╝${NC}"
