#!/usr/bin/env python3
"""
Abstract class that defines properties for all Robots
- abstractmethod - MUST be implemented by derived classes
- virtual methods - CAN be overridden by derived classes
- Regular methods - common to all robots

GAZEBO SENSORS REPO
- https://github.com/gazebosim/gz-sensors/tree/gz-sensors9

GAZEBO DIFF DRIVE
- https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/src/gazebo_ros_diff_drive.cpp

LASER ODOM
https://www.youtube.com/watch?v=HGfJnU2p2YM

"""

import filterpy.kalman
import rclpy
from rclpy.node import Node
from enum import Enum
from abc import ABC, abstractmethod
import math

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, LaserScan, Imu, MagneticField, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion
from filterpy.kalman import KalmanFilter

class RobotState(Enum):
    IDLE = 0
    MOVING = 1
    EXECUTING_TASK = 2
    ERROR = 3
    EMERGENCY = 4

# Robot attributes class equivalent to C++ struct
class RobotAttributes:
    def __init__(self):
        self.robot_type = ""
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta
        self.robot_state = RobotState.IDLE
        self.current_environment = "unknown"  # indoor, outdoor, mixed


class RobotBase(Node, ABC):
    def __init__(self, name, attributes):
        """
        Initialize the robot base class
        
        Args:
            name (str): Name of the robot node
            attributes (RobotAttributes): Robot specific attributes
        """
        super().__init__(name)

        self.robot_attributes = attributes
        
        # Odom from Differential Drive Gazebo Plugin
        self.sub_odom = self.create_subscription(
            Odometry, 
            '/odom', 
            self.cleaned_odom_callback, 
            10
        )
        # Laser odom from ros2 laser scan matcher Gazebo Plugin
        self.sub_laser_odom = self.create_subscription(
            Odometry, 
            '/laser_odom', 
            self.cleaned_odom_callback, 
            10
        )

        # Gazebo Robot 2D LiDAR/"laser" Sensor Plugin
        self.sub_lidar = self.create_subscription(
            LaserScan, 
            '/lidar', 
            self.lidar_callback, 
            10
        )

        # Gazebo Robot IMU Sensor Plugin
        self.sub_imu_sensor = self.create_subscription(
            Imu,
            '/imu',
            self.imu_sensor_callback,
            10
        )

        # Gazebo Robot Depth Camera Sensor Plugin
        self.sub_depth_camera = self.create_subscription(
            Image, 
            '/camera/depth', 
            self.depth_camera_callback, 
            10
        )

        # Gazebo NavSatSensor Plugin
        self.sub_gps = self.create_subscription(
            NavSatFix,
            '/navsat',
            self.gps_callback,
            10
        )

        # Gazebo Magnetic Field Sensor Plugin
        self.sub_magnetometer = self.create_subscription(
            MagneticField,
            '/magnetometer',
            self.magnetometer_sensor_callback,
            10
        )

        # Kalman Filtered precise Odom
        self.pub_raw_odom = self.create_publisher(
            Odometry, 
            'odom_fused', 
            10
        )
   

        self.pub_status = self.create_publisher(
            String, 
            f'/helios/robots/{name}/status', 
            10
        )


        self.navigate_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )


    # Sensor callbacks - to be implemented by derived classes
    def lidar_callback(self, msg):
        """Handle LIDAR data"""
        pass

    def imu_sensor_callback(self, msg):
        """Handle IMU sensor data"""
        pass

    def depth_camera_callback(self, msg):
        """Handle depth camera data"""
        pass
    def magnetometer_sensor_callback(self, msg):
        """Handle IMU sensor data"""
        pass
    def gps_callback(self, msg):
        "Handle GPS Sensor Data"
        pass

    


    @abstractmethod
    def initialize(self):
        """Initialize robot hardware and software - must be implemented by derived classes"""
        pass
    
    @abstractmethod
    def update_sensors(self):
        """Update sensor readings - must be implemented by derived classes"""
        pass
    
    @abstractmethod
    def execute_task(self, task_type, location):
        """
        Execute a specific task at the given location
        
        Args:
            task_type (str): Type of task to execute
            location (PoseStamped): Location to execute task
            
        Returns:
            bool: Success status
        """
        pass
    
    def report_status(self, status):
        """
        Publish robot status to ROS topic
        
        Args:
            status (str): Status message to report
        """
        msg = String()
        msg.data = f"{self.robot_attributes.robot_type}:{status}:{self.current_environment}"
        self.pub_status.publish(msg)
    
    def cleaned_odom_callback(self, msg):
        """
        Handle odometry data updates
        
        Args:
            msg (Odometry): Odometry message from fused sensors
        """
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose
        
        # Update robot attributes with current position
        self.robot_attributes.x_pos = self.current_pose.pose.position.x
        self.robot_attributes.y_pos = self.current_pose.pose.position.y
        
        # Extract theta from quaternion
        q = [
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(q)
        
        self.robot_attributes.theta = yaw
        self.robot_attributes.robot_pose = [
            self.robot_attributes.x_pos, 
            self.robot_attributes.y_pos, 
            self.robot_attributes.theta
        ]
    
    def navigate_to(self, target):
        """
        Navigate robot to target pose
        
        Args:
            target (PoseStamped): Target pose to navigate to
        """
        if not self.navigate_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            return
        
        goal = NavigateToPose.Goal()
        goal.pose = target
        
        self.robot_state = RobotState.MOVING
        self.report_status("NAVIGATING")
        
        self.navigate_client.send_goal_async(
            goal, 
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """
        Handle navigation goal response
        
        Args:
            future: Future object containing goal response
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal was rejected by server')
            self.robot_state = RobotState.IDLE
            return
            
        self.get_logger().info('Goal accepted by server')
        
        # Get result of navigation
        future = goal_handle.get_result_async()
        future.add_done_callback(self.result_callback)
    
    def result_callback(self, future):
        """
        Handle navigation result
        
        Args:
            future: Future object containing navigation result
        """
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # Succeeded
            self.get_logger().info('Navigation successful')
            self.robot_state = RobotState.IDLE
            self.report_status("NAVIGATION_COMPLETE")
        elif status == 5:  # Aborted
            self.get_logger().error('Goal was aborted')
            self.robot_state = RobotState.ERROR
            self.report_status("NAVIGATION_ABORTED")
        elif status == 6:  # Canceled
            self.get_logger().info('Goal was canceled')
            self.robot_state = RobotState.IDLE
            self.report_status("NAVIGATION_CANCELED")
        else:
            self.get_logger().error(f'Unknown result code: {status}')
            self.robot_state = RobotState.ERROR
            self.report_status("NAVIGATION_ERROR")
    
    def feedback_callback(self, feedback_msg):
        """
        Handle navigation feedback
        
        Args:
            feedback_msg: Feedback from navigation action
        """
        feedback = feedback_msg.feedback
        # TODO:Process feedback as needed
    
    def detect_environment(self):
        """
        Detect robot's environment type
        Default implementation - derived classes can override with sensor-based detection
        """
        self.current_environment = "unknown"