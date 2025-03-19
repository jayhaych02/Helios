from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    model = LaunchConfiguration('model')
    robot_description = ParameterValue(
        Command([
            'xacro ',
            'src/helios_description/urdf/robots/',
            model,
            '.urdf.xacro'
        ]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value='firefighter_robot',
            description='Name of the robot model'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        )
    ])