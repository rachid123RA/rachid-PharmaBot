"""
pharmabot_demo.launch.py — Démo soutenance (SANS Nav2)
PharmaBot | Ibn Tofaïl University | 2025-2026

Lance directement : Gazebo + Robot + EDF + Navigation simple
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir  = get_package_share_directory('hospital_robot_spawner')
    world_file = os.path.join(pkg_dir, 'worlds', 'hospital.world')
    robot_sdf  = os.path.join(pkg_dir, 'models', 'pioneer3at', 'model.sdf')

    # ── t+0s : Gazebo ──────────────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    # ── t+3s : Spawn robot à la Pharmacie ──────────────────────────
    spawn = TimerAction(period=3.0, actions=[
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-file', robot_sdf,
                '-entity', 'demo',
                '-x', '1.0', '-y', '16.0', '-z', '0.3',
                '-Y', '-1.5708',
            ],
            output='screen',
        )
    ])

    # ── t+6s : EDF Scheduler ───────────────────────────────────────
    rt_scheduler = TimerAction(period=6.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='rt_scheduler',
            name='rt_scheduler',
            output='screen',
        )
    ])

    # ── t+6s : DMA ─────────────────────────────────────────────────
    dma = TimerAction(period=6.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='dma_tasks',
            name='dma_scheduler',
            output='screen',
        )
    ])

    # ── t+6s : Pharmacist ──────────────────────────────────────────
    pharmacist = TimerAction(period=6.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='pharmacist',
            name='pharmacist_node',
            output='screen',
        )
    ])

    # ── t+7s : Navigation Bridge (contrôleur proportionnel) ────────
    nav_bridge = TimerAction(period=7.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='nav_bridge',
            name='navigation_bridge',
            output='screen',
        )
    ])

    # ── t+7s : Mission Manager ─────────────────────────────────────
    mission = TimerAction(period=7.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
        )
    ])

    # ── t+8s : Watchdog ────────────────────────────────────────────
    watchdog = TimerAction(period=8.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='watchdog_node',
            name='pharmabot_watchdog',
            output='screen',
        )
    ])

    # ── t+9s : Doctor Request (scénario EDF auto) ──────────────────
    doctor = TimerAction(period=9.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='doctor_request',
            name='doctor_request_node',
            output='screen',
        )
    ])

    # ── t+10s : Dashboard ──────────────────────────────────────────
    dashboard = TimerAction(period=10.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='dashboard_p4',
            name='visual_dashboard_p4',
            output='screen',
        )
    ])

    return LaunchDescription([
        gazebo,
        spawn,
        rt_scheduler,
        dma,
        pharmacist,
        nav_bridge,
        mission,
        watchdog,
        doctor,
        dashboard,
    ])
