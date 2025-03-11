#!/usr/bin/env python3
"""
Wheel odometry node for differential drive robots.

This node computes odometry from wheel encoders for differential drive robots,
publishing odometry messages and TF transforms.

Author: Jaden Howard 
Contact: jaseanhow@gmail.com tun85812@temple.edu
Date: March 11 2025
"""
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

# Import kinematics - adjust path as needed for your setup
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), '..', '..'))
from helios_utils import wheel_odom_kinematics as kn

LEFT = 0
RIGHT = 1
X = 0
Y = 1
THETA = 2

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        
        
        """
        # Declare parameters
        self.declare_parameter('tb3_model', 'burger')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        # Get parameters
        robot_model = self.get_parameter('tb3_model').value
        
        # Robot parameters
        if robot_model == 'burger':
            self.wheel_separation = 0.160  # [m]
            self.turning_radius = 0.080    # [m]
            self.robot_radius = 0.105      # [m]
        elif robot_model == 'waffle' or robot_model == 'waffle_pi':
            self.wheel_separation = 0.287  # [m]
            self.turning_radius = 0.1435   # [m]
            self.robot_radius = 0.220      # [m]
        else:
            self.get_logger().error(f'Turtlebot3 model {robot_model} not defined')
            
        """


        self.wheel_radius = 0.033  # [m]
        # Joint states data
        self.prev_joint_states = None

        # QoS profile
        qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE,history=QoSHistoryPolicy.KEEP_LAST,depth=10)

        self.odom = Odometry()
        self.odom.header.frame_id = self.get_parameter('odom_frame').value
        self.odom.child_frame_id = self.get_parameter('base_frame').value
        self.odom.pose.covariance = np.diag((0.1, 0.1, 1e6, 1e6, 1e6, 0.2)).flatten().tolist()
        self.odom.twist.covariance = np.diag((0.1, 0.1, 1e6, 1e6, 1e6, 0.2)).flatten().tolist()
        self.pose = [0.0, 0.0, 0.0]  # (x, y, theta)

        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.joint_states_sub = self.create_subscription(JointState,'joint_states',self.joint_states_callback,qos)

        self.get_logger().info('Wheel Odometry node initialized')

    def joint_states_callback(self, msg: JointState) -> None:
        # Check if first message received
        if self.prev_joint_states is None:
            self.prev_joint_states = msg
            return
            
        # Update and send messages
        self.update_odometry(msg)
        self.update_tf()
        
        # Save message
        self.prev_joint_states = msg

    def update_odometry(self, new_joint_states: JointState) -> None:
        # Calculate change in wheel angles
        delta_wheel_l, delta_wheel_r, delta_time = kn.calculate_wheel_change(new_joint_states, self.prev_joint_states)

        # Calculate displacement
        delta_s, delta_theta = kn.calculate_displacement( delta_wheel_l, delta_wheel_r, self.wheel_radius, self.wheel_separation)

        # Compute new pose
        self.pose = kn.calculate_pose(self.pose, delta_s, delta_theta)

        # Update odometry header timestamp
        # Note: Using ROS2's clock instead of adding duration
        self.odom.header.stamp = self.get_clock().now().to_msg()

        # Update position and orientation
        self.odom.pose.pose.position.x = self.pose[0]
        self.odom.pose.pose.position.y = self.pose[1]
        self.odom.pose.pose.orientation.z = np.sin(self.pose[2]/2)
        self.odom.pose.pose.orientation.w = np.cos(self.pose[2]/2)

        # Update velocity
        linear_vel = delta_s / delta_time if delta_time > 0 else 0.0
        angular_vel = delta_theta / delta_time if delta_time > 0 else 0.0
        self.odom.twist.twist.linear.x = linear_vel
        self.odom.twist.twist.angular.z = angular_vel

        self.odom_pub.publish(self.odom)

    def update_tf(self) -> None:
        odom_tf = TransformStamped()

        odom_tf.header.stamp = self.odom.header.stamp
        odom_tf.header.frame_id = self.odom.header.frame_id
        odom_tf.child_frame_id = self.odom.child_frame_id

        odom_tf.transform.translation.x = self.pose[X]
        odom_tf.transform.translation.y = self.pose[Y]
        odom_tf.transform.rotation.z = self.odom.pose.pose.orientation.z
        odom_tf.transform.rotation.w = self.odom.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(odom_tf)

def main(args=None):
    rclpy.init(args=args)
    wheel_odom_node = WheelOdometry()
    
    try:
        rclpy.spin(wheel_odom_node)
    except KeyboardInterrupt:
        pass
    finally:
        wheel_odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()