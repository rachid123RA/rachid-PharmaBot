"""
pharmabot_nodes_only.launch.py — Nœuds ROS2 sans Gazebo
Utiliser avec gz-harmonic lancé séparément (ARM64 / UTM)

Terminal 1 : ros2 launch hospital_robot_spawner pharmabot_nodes_only.launch.py
Terminal 2 : gz sim hospital.world
"""
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    # t+0s : EDF Scheduler
    rt_scheduler = Node(
        package='hospital_robot_spawner',
        executable='rt_scheduler',
        name='rt_scheduler',
        output='screen',
    )

    # t+0s : DMA
    dma = Node(
        package='hospital_robot_spawner',
        executable='dma_tasks',
        name='dma_scheduler',
        output='screen',
    )

    # t+0s : Pharmacist
    pharmacist = Node(
        package='hospital_robot_spawner',
        executable='pharmacist',
        name='pharmacist_node',
        output='screen',
    )

    # t+1s : Navigation Bridge (contrôleur proportionnel)
    nav_bridge = TimerAction(period=1.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='nav_bridge',
            name='navigation_bridge',
            output='screen',
        )
    ])

    # t+1s : Mission Manager
    mission = TimerAction(period=1.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='mission_manager',
            name='mission_manager',
            output='screen',
        )
    ])

    # t+2s : Watchdog
    watchdog = TimerAction(period=2.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='watchdog_node',
            name='pharmabot_watchdog',
            output='screen',
        )
    ])

    # t+3s : Doctor Request (scénario EDF automatique)
    doctor = TimerAction(period=3.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='doctor_request',
            name='doctor_request_node',
            output='screen',
        )
    ])

    # t+4s : Dashboard P4 (affichage principal)
    dashboard = TimerAction(period=4.0, actions=[
        Node(
            package='hospital_robot_spawner',
            executable='dashboard_p4',
            name='visual_dashboard_p4',
            output='screen',
        )
    ])

    return LaunchDescription([
        rt_scheduler,
        dma,
        pharmacist,
        nav_bridge,
        mission,
        watchdog,
        doctor,
        dashboard,
    ])
