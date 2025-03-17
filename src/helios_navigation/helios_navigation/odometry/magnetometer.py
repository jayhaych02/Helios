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

import numpy as np

class Magnetometer(Node):
    def __init__(self):
        super().__init__('location_magnetometer')

        # Robot base class subs to '/odom/magnetometer', so publish to this topic