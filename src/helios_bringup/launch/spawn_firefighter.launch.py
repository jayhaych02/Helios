from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_gazebo = LaunchConfiguration('use_gazebo')
    robot_name = LaunchConfiguration('robot_name', default='firefighter_robot')
    robot_description = LaunchConfiguration(
        'robot_description',
        default=['package://helios_description/urdf/robots/', robot_name, '.urdf.xacro'].join('')
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            description='Whether to start Gazebo'
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-name', robot_name, '-file', robot_description],
            output='screen',
            condition=LaunchConfigurationEquals('use_gazebo', 'true')
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ])