#!/usr/bin/env python3
"""
Abstract class that defines properties for all Robots
- abstractmethod - MUST be implemented by derived classes
- virtual methods - CAN be overridden by derived classes
- Regular methods - common to all robots
"""

import rclpy
from rclpy.node import Node
from enum import Enum
from abc import ABC, abstractmethod
import math

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf_transformations import euler_from_quaternion

# Robot state enum
class RobotState(Enum):
    IDLE = 0
    MOVING = 1
    EXECUTING_TASK = 2
    ERROR = 3
    EMERGENCY = 4

# Robot attributes class equivalent to C++ struct
class RobotAttributes:
    def __init__(self):
        self.robot_id = ""
        self.robot_type = ""
        self.max_speed = 0.0
        self.max_payload = 0.0
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0
        self.robot_pose = [0.0, 0.0, 0.0]  # x, y, theta


class RobotBase(Node, ABC):
    def __init__(self, name, attributes):
        """
        Initialize the robot base class
        
        Args:
            name (str): Name of the robot node
            attributes (RobotAttributes): Robot specific attributes
        """
        super().__init__(name)
        
        # Core attributes
        self.robot_attributes = attributes
        self.robot_state = RobotState.IDLE
        self.battery_level = 100.0
        self.current_pose = PoseStamped()
        self.current_environment = "unknown"  # indoor, outdoor, mixed
        
        # Core Sensors
        self.sub_lidar = self.create_subscription(
            LaserScan, 
            'scan', 
            self.lidar_callback, 
            10
        )
        
        self.sub_depth_camera = self.create_subscription(
            Image, 
            'depth_camera', 
            self.depth_camera_callback, 
            10
        )
        
        self.sub_gas_sensor = self.create_subscription(
            Float64, 
            'gas_sensor', 
            self.gas_sensor_callback, 
            10
        )
        
        self.sub_thermal_camera = self.create_subscription(
            Image, 
            'thermal_camera', 
            self.thermal_camera_callback, 
            10
        )
        
        # Odometry and localization
        self.pub_raw_odom = self.create_publisher(
            Odometry, 
            'raw_odom', 
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, 
            '/helios/odom', 
            self.odom_callback, 
            10
        )
        
        # Status publisher
        self.pub_status = self.create_publisher(
            String, 
            f'/helios/robots/{name}/status', 
            10
        )
        
        # Navigation client
        self.navigate_client = ActionClient(
            self, 
            NavigateToPose, 
            'navigate_to_pose'
        )

    # Sensor callbacks - to be implemented by derived classes
    def lidar_callback(self, msg):
        """Handle LIDAR data"""
        pass
        
    def depth_camera_callback(self, msg):
        """Handle depth camera data"""
        pass
        
    def gas_sensor_callback(self, msg):
        """Handle gas sensor data"""
        pass
        
    def thermal_camera_callback(self, msg):
        """Handle thermal camera data"""
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
        msg.data = f"{self.robot_attributes.robot_id}:{self.robot_attributes.robot_type}:{status}:{self.current_environment}"
        self.pub_status.publish(msg)
    
    def odom_callback(self, msg):
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
        # Process feedback as needed
    
    def detect_environment(self):
        """
        Detect robot's environment type
        Default implementation - derived classes can override with sensor-based detection
        """
        self.current_environment = "unknown"