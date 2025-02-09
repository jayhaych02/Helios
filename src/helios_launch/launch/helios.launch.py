
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
   return LaunchDescription([
       Node(
           package='helios_testing',
           executable='test_unity_node',
           name='unity_test_pub',
           output='screen',
           parameters=[{
               'use_sim_time': False
           }]
       )
   ])