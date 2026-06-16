"""
pharmabot_vm.launch.py — Lancement complet PharmaBot pour VM Linux
PharmaBot | Ibn Tofaïl University | 2025-2026 | Sans Docker

Ordre de démarrage :
    t+0s   Gazebo Classic (hospital.world)
    t+4s   Map Server (carte hôpital)
    t+5s   Nav2 (AMCL + planner + controller + costmap)
    t+8s   EDF RT Scheduler
    t+8s   DMA Scheduler
    t+8s   Pharmacist Node
    t+9s   Mission Manager
    t+9s   Watchdog Node
    t+10s  Nav2 Navigator (remplace navigation_bridge)
    t+10s  Delivery Detector
    t+12s  Doctor Request Node
    t+14s  Dashboard P4

Commande unique de lancement :
    ros2 launch hospital_robot_spawner pharmabot_vm.launch.py
"""
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir  = get_package_share_directory('hospital_robot_spawner')
    nav2_dir = get_package_share_directory('nav2_bringup')

    world_file = os.path.join(pkg_dir, 'worlds', 'hospital.world')
    map_file   = os.path.join(pkg_dir, 'maps', 'hospital_map.yaml')
    nav2_cfg   = os.path.join(pkg_dir, 'config', 'nav2_params.yaml')

    robot_sdf = os.path.join(pkg_dir, 'models', 'pioneer3at', 'model.sdf')

    # ── Gazebo Classic t+0s ──────────────────────────────────────
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose', world_file,
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    # ── Spawn robot Pioneer 3AT t+2s ─────────────────────────────
    spawn_robot = TimerAction(period=2.0, actions=[
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

    # ── Map Server t+4s ──────────────────────────────────────────
    map_server = TimerAction(period=4.0, actions=[
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'yaml_filename': map_file,
            }],
        )
    ])

    # ── Lifecycle Manager Map t+4.5s ─────────────────────────────
    lifecycle_map = TimerAction(period=4.5, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': ['map_server'],
            }],
        )
    ])

    # ── Nav2 (AMCL + planner + controller) t+5s ─────────────────
    nav2 = TimerAction(period=5.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'True',
                'params_file':  nav2_cfg,
                'autostart':    'True',
            }.items(),
        )
    ])

    # ── AMCL t+5s ────────────────────────────────────────────────
    amcl = TimerAction(period=5.0, actions=[
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_cfg, {'use_sim_time': True}],
            remappings=[('scan', '/demo/scan')],
        )
    ])

    # ── Lifecycle Manager Nav2 t+6s ──────────────────────────────
    lifecycle_nav = TimerAction(period=6.0, actions=[
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'autostart': True,
                'node_names': [
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'behavior_server',
                ],
            }],
        )
    ])

    # ── EDF RT Scheduler t+8s ────────────────────────────────────
    rt_scheduler = TimerAction(period=8.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='rt_scheduler',
            name='rt_scheduler',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── DMA Scheduler t+8s ───────────────────────────────────────
    dma = TimerAction(period=8.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='dma_tasks',
            name='dma_scheduler',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── Pharmacist Node t+8s ─────────────────────────────────────
    pharmacist = TimerAction(period=8.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='pharmacist',
            name='pharmacist_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── Mission Manager t+9s ─────────────────────────────────────
    mission = TimerAction(period=9.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── Watchdog t+9s ────────────────────────────────────────────
    watchdog = TimerAction(period=9.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='watchdog_node',
            name='pharmabot_watchdog',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── Nav2 Navigator t+10s (remplace navigation_bridge) ────────
    nav2_navigator = TimerAction(period=10.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='nav2_navigator',
            name='nav2_navigator',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── Delivery Detector t+10s ──────────────────────────────────
    delivery = TimerAction(period=10.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='delivery_detect',
            name='delivery_detector',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── Doctor Request Node t+12s ────────────────────────────────
    doctor = TimerAction(period=12.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='doctor_request',
            name='doctor_request_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    # ── Dashboard P4 t+14s ───────────────────────────────────────
    dashboard = TimerAction(period=14.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='dashboard_p4',
            name='visual_dashboard_p4',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

    return LaunchDescription([
        gazebo,
        spawn_robot,
        map_server,
        lifecycle_map,
        nav2,
        amcl,
        lifecycle_nav,
        rt_scheduler,
        dma,
        pharmacist,
        mission,
        watchdog,
        nav2_navigator,
        delivery,
        doctor,
        dashboard,
    ])
