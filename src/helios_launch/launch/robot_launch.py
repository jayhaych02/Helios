import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher

import time
def generate_launch_description():
    world_path = os.path.expanduser('~/ros2_ws/src/helios_webots/worlds/basicworld.wbt')
    webots = WebotsLauncher(world=world_path)
    return LaunchDescription([
        webots,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.ExecuteProcess(cmd=['sleep', '2']),  
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])