#!/usr/bin/env python3
"""
pharmabot_ignition.launch.py — Lancement PharmaBot sur Ignition Gazebo Fortress

Compatible Ubuntu 22.04 ARM64 + ROS2 Humble + Ignition Fortress

Ordre de démarrage :
  t+0s  → Ignition Gazebo (hospital_ignition.world — hôpital 3D complet)
  t+2s  → ROS2-Ignition Bridge (/demo/cmd_vel ↔ /demo/odom ↔ /demo/scan)
  t+4s  → EDF Scheduler (ordonnancement temps réel HARD/SOFT/FIRM_RT)
  t+5s  → Mission Manager + Pharmacist Node
  t+6s  → Navigation Bridge (contrôle robot via /demo/cmd_vel)
  t+7s  → Delivery Detector (détection arrivée via /demo/odom)
  t+8s  → Doctor Request (génère les requêtes médicales automatiquement)
  t+9s  → Watchdog Node + Dashboard
  t+10s → Gazebo Bridge (médicaments 3D — spawn/transport/dépôt)
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg = get_package_share_directory("hospital_robot_spawner")
    world = os.path.join(pkg, "worlds", "hospital_ignition.world")

    # ── t+0s : Ignition Gazebo (hôpital 3D) ──────────────────────────────
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "--verbose", "1", world],
        output="screen",
        additional_env={"IGN_GAZEBO_SYSTEM_PLUGIN_PATH": "/usr/lib/x86_64-linux-gnu/ign-gazebo-6/plugins:/usr/lib/aarch64-linux-gnu/ign-gazebo-6/plugins"},
    )

    # ── t+2s : Bridge ROS2 ↔ Ignition ────────────────────────────────────
    # cmd_vel : ROS2→Ignition  (] = ros2 vers ign)
    # odom    : Ignition→ROS2  ([ = ign vers ros2)
    # scan    : Ignition→ROS2
    # clock   : Ignition→ROS2
    bridge = TimerAction(period=2.0, actions=[
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="pharmabot_bridge",
            arguments=[
                "/demo/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
                "/demo/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
                "/demo/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
                "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            ],
            output="screen",
        )
    ])

    # ── t+4s : EDF Scheduler ─────────────────────────────────────────────
    rt_scheduler = TimerAction(period=4.0, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="rt_scheduler",
            name="rt_scheduler",
            output="screen",
        )
    ])

    # ── t+5s : DMA Tasks ─────────────────────────────────────────────────
    dma = TimerAction(period=5.0, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="dma_tasks",
            name="dma_tasks",
            output="screen",
        )
    ])

    # ── t+5.5s : Pharmacist Node ──────────────────────────────────────────
    pharmacist = TimerAction(period=5.5, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="pharmacist",
            name="pharmacist_node",
            output="screen",
        )
    ])

    # ── t+6s : Mission Manager ────────────────────────────────────────────
    mission = TimerAction(period=6.0, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="mission_manager",
            name="mission_manager",
            output="screen",
        )
    ])

    # ── t+6.5s : Navigation Bridge ────────────────────────────────────────
    nav_bridge = TimerAction(period=6.5, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="nav_bridge",
            name="navigation_bridge",
            output="screen",
        )
    ])

    # ── t+7s : Delivery Detector ──────────────────────────────────────────
    delivery = TimerAction(period=7.0, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="delivery_detect",
            name="delivery_detector",
            output="screen",
        )
    ])

    # ── t+8s : Doctor Request (génère les requêtes) ───────────────────────
    doctor = TimerAction(period=8.0, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="doctor_request",
            name="doctor_request_node",
            output="screen",
        )
    ])

    # ── t+9s : Watchdog ───────────────────────────────────────────────────
    watchdog = TimerAction(period=9.0, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="watchdog_node",
            name="watchdog_node",
            output="screen",
        )
    ])

    # ── t+9.5s : Dashboard ────────────────────────────────────────────────
    dashboard = TimerAction(period=9.5, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="dashboard",
            name="dashboard",
            output="screen",
        )
    ])

    # ── t+10s : Gazebo Bridge 3D (médicaments physiques) ──────────────────
    gazebo_bridge = TimerAction(period=10.0, actions=[
        Node(
            package="hospital_robot_spawner",
            executable="gazebo_bridge",
            name="pharmabot_gazebo_bridge",
            output="screen",
        )
    ])

    return LaunchDescription([
        gazebo,
        bridge,
        rt_scheduler,
        dma,
        pharmacist,
        mission,
        nav_bridge,
        delivery,
        doctor,
        watchdog,
        dashboard,
        gazebo_bridge,
    ])
