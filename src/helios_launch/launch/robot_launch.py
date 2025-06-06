from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    helios_sim_share = FindPackageShare('helios_sim')
    helios_control_share = FindPackageShare('helios_control')

    world_file = PathJoinSubstitution([helios_sim_share, 'worlds', 'warehouse.sdf'])
    bridge_config = PathJoinSubstitution([helios_control_share, 'config', 'rosbridge_topics.yaml'])

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_file],
        output='screen'
    )

   
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen'
    )

    command_tugbot = Node(
        package='helios_control',
        executable='tugbot_controller',
        name='tugbot_cmdvel',
        output='screen'
    )

    
    return LaunchDescription([
        gazebo,
        bridge,
        command_tugbot
    ])