# PharmaBot — Image ROS2 Humble (ARM64 compatible Mac Apple Silicon)
FROM osrf/ros:humble-desktop

# Éviter les interactions pendant apt
ENV DEBIAN_FRONTEND=noninteractive

# Installer les dépendances système
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Installer les dépendances Python
RUN pip3 install --no-cache-dir \
    "stable-baselines3==2.3.2" \
    "gymnasium==0.29.1" \
    "numpy<2.0" \
    "tensorboard" \
    "optuna==3.6.1"

# Créer le workspace ROS2
WORKDIR /ros2_ws

# Copier le code source
COPY hospital_robot_spawner/ src/hospital_robot_spawner/
COPY tests/ tests/

# Builder le package ROS2
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    colcon build --packages-select hospital_robot_spawner 2>/dev/null || true"

# Sourcer l'environnement automatiquement
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Point d'entrée par défaut
CMD ["/bin/bash"]
