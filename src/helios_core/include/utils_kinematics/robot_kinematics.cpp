/**
 * @brief Kinematics for Wheel Odometry
 * @note Helios will use 2 types of Odometry : (1) Motion Model & (2) Laser 
 * @note Look at Mobile Robotics repo for Wheel Odometry 
 * @link https://github.com/jayhaych02/MobileRobotics/blob/c32c7504e9cbf0e30fd8f85f20ec760d898229be/src/wheel_odometry/src/wheel_odometry/wheel_odometry.py#L21
 * @note Using Wheel Odom is still on the table, but looking like the other 2 are more robust
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <array>
#include <cmath>
#include "robot_kinematics.hpp"

namespace helios_kinematics {

/**
 * @brief Calculate wheel angle changes and time delta
 * @param new_joint_states New joint states
 * @param prev_joint_states Previous joint states
 * @return std::array<double, 3> (delta_wheel_l, delta_wheel_r, delta_time)
 */
std::array<double, 3> calculate_wheel_change(const sensor_msgs::msg::JointState& new_joint_states,const sensor_msgs::msg::JointState& prev_joint_states)
{
    // Get wheel angle changes
    double delta_wheel_l = new_joint_states.position[LEFT] - prev_joint_states.position[LEFT];
    double delta_wheel_r = new_joint_states.position[RIGHT] - prev_joint_states.position[RIGHT];
    
    // Get time change
    double delta_time = 
        (new_joint_states.header.stamp.sec + new_joint_states.header.stamp.nanosec * 1e-9) -
        (prev_joint_states.header.stamp.sec + prev_joint_states.header.stamp.nanosec * 1e-9);
    
    // Data validation
    if (std::isnan(delta_wheel_l)) {
        delta_wheel_l = 0.0;
    }
    if (std::isnan(delta_wheel_r)) {
        delta_wheel_r = 0.0;
    }
    
    return {delta_wheel_l, delta_wheel_r, delta_time};
}

/**
 * @brief Calculate displacement from wheel rotations
 * @param delta_wheel_l Change in left wheel angle [rad]
 * @param delta_wheel_r Change in right wheel angle [rad]
 * @param wheel_radius Wheel radius [m]
 * @param wheel_separation Wheel separation [m]
 * @return std::array<double, 2> (delta_s, delta_theta)
 */
std::array<double, 2> calculate_displacement(double delta_wheel_l, double delta_wheel_r, double wheel_radius, double wheel_separation)
{
    // Calculate angular displacement
    double angular = (wheel_radius / wheel_separation) * (delta_wheel_r - delta_wheel_l);
    
    // Calculate linear displacement
    double linear = (wheel_radius / 2.0) * (delta_wheel_r + delta_wheel_l);
    
    return {linear, angular};
}

/**
 * @brief Calculate new pose from previous pose and displacement
 * @param prev_pose Previous pose (x, y, theta) [m, m, rad]
 * @param delta_s Linear displacement [m]
 * @param delta_theta Angular displacement [rad]
 * @return std::array<double, 3> New pose (x, y, theta) [m, m, rad]
 */
std::array<double, 3> calculate_pose(const std::array<double, 3>& prev_pose, double delta_s, double delta_theta)
{
    double new_theta = prev_pose[THETA] + delta_theta;
    
    // Apply kinematics model (using average orientation)
    double x_new = prev_pose[X] + delta_s * std::cos(prev_pose[THETA] + 0.5 * delta_theta);
    double y_new = prev_pose[Y] + delta_s * std::sin(prev_pose[THETA] + 0.5 * delta_theta);
    
    return {x_new, y_new, new_theta};
}

} // namespace helios_kinematics