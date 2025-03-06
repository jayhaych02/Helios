import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Write the simplified URDF directly to a file
    simplified_urdf_path = '/tmp/firefighter_robot.urdf'
    
    # Create directories
    home_dir = os.path.expanduser('~')
    pkg_dir = os.path.join(home_dir, 'robotics', 'Helios', 'src')
    urdf_dir = os.path.join(pkg_dir, 'helios_description', 'urdf', 'robots')
    os.makedirs(urdf_dir, exist_ok=True)
    
    # Path to the firefighter URDF
    urdf_path = os.path.join(urdf_dir, 'simplified_firefighter.urdf')
    
    # Copy the simplified URDF file to the destination
    copy_urdf = ExecuteProcess(
        cmd=['bash', '-c', f'cp {urdf_path} {simplified_urdf_path}'],
        output='screen'
    )
    
    # Launch Gazebo with empty world
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )
    
    # Spawn the robot
    spawn = ExecuteProcess(
        cmd=['gz', 'service', '-s', '/world/empty/create',
             '--reqtype', 'gz.msgs.EntityFactory',
             '--reptype', 'gz.msgs.Boolean',
             '--timeout', '300',
             '--req', f'sdf_filename: "{simplified_urdf_path}" name: "firefighter_robot" pose: {{position: {{x: 0.0, y: 0.0, z: 0.3}}}}'],
        output='screen'
    )
    
    # ROS 2 bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        output='screen'
    )
    
    # Delay to ensure Gazebo is running before spawning robot
    delayed_spawn = TimerAction(
        period=5.0,
        actions=[spawn]
    )
    
    # Delay to ensure robot is spawned before starting bridge
    delayed_bridge = TimerAction(
        period=8.0,
        actions=[bridge]
    )

    return LaunchDescription([
        copy_urdf,
        gazebo,
        delayed_spawn,
        delayed_bridge
    ])