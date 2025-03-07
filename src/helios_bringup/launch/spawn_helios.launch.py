#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    pkg_share = get_package_share_directory('helios_description')
    
    # Define paths based on your directory structure
    default_model_path = os.path.join(pkg_share, 'urdf/robots/helios.urdf.xacro')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    world_path = os.path.join(pkg_share, 'worlds/empty_world.world')
    
    # Launch configuration variables specific to simulation
    gui = LaunchConfiguration('gui')
    model = LaunchConfiguration('model')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_gazebo = LaunchConfiguration('use_gazebo')
    
    # Declare the launch arguments
    declare_model_path_cmd = DeclareLaunchArgument(
        'model',
        default_value=default_model_path,
        description='Absolute path to robot urdf file')
        
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
        
    declare_use_robot_state_pub_cmd = DeclareLaunchArgument(
        'use_robot_state_pub',
        default_value='True',
        description='Whether to start the robot state publisher')
        
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')
        
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true')
        
    declare_use_gazebo_cmd = DeclareLaunchArgument(
        'use_gazebo',
        default_value='True',
        description='Whether to start Gazebo')
        
    declare_gui_cmd = DeclareLaunchArgument(
        'gui',
        default_value='True',
        description='Flag to enable joint_state_publisher_gui')
    
    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_gazebo),
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # Start Gazebo client
    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')]),
        condition=IfCondition(use_gazebo)
    )
    
    # Create the Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', model]),
                    'use_sim_time': use_sim_time}]
    )
    
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'helios',
                '-topic', 'robot_description'],
        output='screen'
    )
    
    # Joint state publisher node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(PythonExpression(['not ', gui]))
    )
    
    # Joint state publisher gui node
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(gui)
    )
    
    # RVIZ
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )
    
    # Define a simple teleop node for keyboard control
    teleop_node = Node(
        package='helios_description',
        executable='teleop_keyboard.py',
        name='teleop_keyboard',
        output='screen'
    )
    
    # Return the LaunchDescription
    return LaunchDescription([
        declare_model_path_cmd,
        declare_rviz_config_file_cmd,
        declare_use_robot_state_pub_cmd,
        declare_use_rviz_cmd,
        declare_use_sim_time_cmd,
        declare_use_gazebo_cmd,
        declare_gui_cmd,
        
        # Start Gazebo
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        
        # Launch robot components
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        spawn_entity,
        rviz_node,
        teleop_node
    ])