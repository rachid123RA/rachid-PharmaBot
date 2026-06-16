#!/bin/bash
# ══════════════════════════════════════════════════════════════
#  run_visual.sh — Visualisation 2D PharmaBot pour soutenance
#  Ne nécessite PAS Gazebo — fonctionne sur ARM64 (UTM/VM)
#  Lance automatiquement en mode démo si ROS2 n'est pas actif
# ══════════════════════════════════════════════════════════════

echo ""
echo "╔══════════════════════════════════════════════════╗"
echo "║   PharmaBot — Visualisation 2D (Soutenance)     ║"
echo "║   Ibn Tofaïl University | ROS2 Humble           ║"
echo "╚══════════════════════════════════════════════════╝"
echo ""

# Installer matplotlib si absent
python3 -c "import matplotlib" 2>/dev/null || {
    echo "[INSTALL] Installation matplotlib..."
    pip3 install matplotlib --quiet
}

# Source ROS2 si disponible
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "[OK] ROS2 Humble sourcé"
fi

# Source workspace si disponible
if [ -f ~/ros2_ws/install/setup.bash ]; then
    source ~/ros2_ws/install/setup.bash
    echo "[OK] Workspace sourcé"
fi

# Activer OpenGL software rendering pour VM
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3

# Forcer backend Tk (disponible partout sous Ubuntu Desktop)
export MPLBACKEND=TkAgg

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ""
echo "  ▶ Lancement visualisation..."
echo "  (Fermer la fenêtre pour quitter)"
echo ""

python3 "$SCRIPT_DIR/demo_visual.py"
