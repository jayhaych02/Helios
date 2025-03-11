#!/usr/bin/env python3

from sensor_msgs.msg import JointState
import numpy as np
from typing import List, Tuple

# Indexing values
LEFT = 0
RIGHT = 1
X = 0
Y = 1
THETA = 2

def calculate_wheel_change(new_joint_states: JointState, prev_joint_states: JointState) -> Tuple[float, float, float]:
    """
    Calculate wheel angle changes and time delta
    
    Args:
        new_joint_states: New joint states
        prev_joint_states: Previous joint states
    
    Returns:
        Tuple containing (delta_wheel_l, delta_wheel_r, delta_time)
    """
    delta_wheel_l = new_joint_states.position[LEFT] - prev_joint_states.position[LEFT]
    delta_wheel_r = new_joint_states.position[RIGHT] - prev_joint_states.position[RIGHT]
    
    # Convert ROS2 time to seconds
    new_time_sec = new_joint_states.header.stamp.sec + new_joint_states.header.stamp.nanosec * 1e-9
    prev_time_sec = prev_joint_states.header.stamp.sec + prev_joint_states.header.stamp.nanosec * 1e-9
    delta_time = new_time_sec - prev_time_sec
    
    # Data validation
    if np.isnan(delta_wheel_l):
        delta_wheel_l = 0.0
    if np.isnan(delta_wheel_r):
        delta_wheel_r = 0.0
    
    return (delta_wheel_l, delta_wheel_r, delta_time)

def calculate_displacement(delta_wheel_l: float, delta_wheel_r: float, wheel_radius: float, wheel_separation: float) -> Tuple[float, float]:
    """
    Calculate displacement from wheel rotations
    
    Args:
        delta_wheel_l: Change in left wheel angle [rad]
        delta_wheel_r: Change in right wheel angle [rad]
        wheel_radius: Wheel radius [m]
        wheel_separation: Wheel separation [m]
    
    Returns:
        Tuple containing (delta_s, delta_theta)
    """
    angular = (wheel_radius / wheel_separation) * (delta_wheel_r - delta_wheel_l)
    linear = (wheel_radius / 2) * (delta_wheel_r + delta_wheel_l)
    
    delta_s = linear  # Linear displacement [m]
    delta_theta = angular  # Angular displacement [rad]
    
    return (delta_s, delta_theta)

def calculate_pose(prev_pose: List[float], delta_s: float, delta_theta: float) -> np.ndarray:
    """
    Calculate new pose from previous pose and displacement
    
    Args:
        prev_pose: Previous pose (x, y, theta) [m, m, rad]
        delta_s: Linear displacement [m]
        delta_theta: Angular displacement [rad]
    
    Returns:
        Array containing new pose (x, y, theta) [m, m, rad]
    """
    new_theta = prev_pose[THETA] + delta_theta
    
    # Use the midpoint orientation for better accuracy
    theta_mid = prev_pose[THETA] + 0.5 * delta_theta
    
    x_new = prev_pose[X] + delta_s * np.cos(theta_mid)
    y_new = prev_pose[Y] + delta_s * np.sin(theta_mid)
    
    pose = np.array([x_new, y_new, new_theta])
    
    return pose