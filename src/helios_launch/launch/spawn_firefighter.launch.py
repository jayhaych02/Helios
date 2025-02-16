import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Get paths
    pkg_dir = os.path.join(os.getcwd(), 'src')
    world_file = os.path.join(pkg_dir, 'helios_gazebo', 'worlds', 'forest_fire_world.sdf')
    urdf_file = os.path.join(pkg_dir, 'helios_description', 'urdf', 'robots', 'firefighter_robot.urdf.xacro')

    # Launch Gazebo
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', world_file],
        output='screen'
    )

    # Generate URDF first
    generate_urdf = ExecuteProcess(
        cmd=['xacro', urdf_file, '>', '/tmp/test_robot.urdf'],
        shell=True,
        output='screen'
    )

    # Spawn robot - using exactly what worked in terminal
    spawn = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/forest_fire_scenario/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '300',
             '--req', 'sdf_filename: "/tmp/test_robot.urdf" name: "firefighter_robot" pose: {position: {x: 0.0, y: 0.0, z: 0.5}}'],
        output='screen'
    )

    # Add a small delay to ensure Gazebo is fully loaded before spawning
    delayed_spawn = TimerAction(
        period=2.0,
        actions=[generate_urdf, spawn]
    )

    return LaunchDescription([
        gazebo,
        delayed_spawn
    ])