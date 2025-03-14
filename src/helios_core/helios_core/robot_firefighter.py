import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import numpy as np
import sys
import os
import robot_base as robot_base

class RobotFireFighter(Node,robot_base): 
    def __init__(self):
        super().__init__('robot_firefighter')
    

