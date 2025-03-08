#ifndef ROBOT_KINEMATICS_HPP
#define ROBOT_KINEMATICS_HPP

#include "sensor_msgs/msg/joint_state.hpp"
#include <array>
#include <cmath>

namespace helios_kinematics {

// Indexing values
constexpr int LEFT = 0;
constexpr int RIGHT = 1;
constexpr int X = 0;
constexpr int Y = 1;
constexpr int THETA = 2;

/**
 * @brief Calculate wheel angle changes and time delta
 * @param new_joint_states New joint states
 * @param prev_joint_states Previous joint states
 * @return std::array<double, 3> (delta_wheel_l, delta_wheel_r, delta_time)
 */
std::array<double, 3> calculate_wheel_change(const sensor_msgs::msg::JointState& new_joint_states,const sensor_msgs::msg::JointState& prev_joint_states);

/**
 * @brief Calculate displacement from wheel rotations
 * @param delta_wheel_l Change in left wheel angle [rad]
 * @param delta_wheel_r Change in right wheel angle [rad]
 * @param wheel_radius Wheel radius [m]
 * @param wheel_separation Wheel separation [m]
 * @return std::array<double, 2> (delta_s, delta_theta)
 */
std::array<double, 2> calculate_displacement(double delta_wheel_l, double delta_wheel_r, double wheel_radius, double wheel_separation);

/**
 * @brief Calculate new pose from previous pose and displacement
 * @param prev_pose Previous pose (x, y, theta) [m, m, rad]
 * @param delta_s Linear displacement [m]
 * @param delta_theta Angular displacement [rad]
 * @return std::array<double, 3> New pose (x, y, theta) [m, m, rad]
 */
std::array<double, 3> calculate_pose(const std::array<double, 3>& prev_pose, double delta_s, double delta_theta);

} // namespace helios_kinematics

#endif // ROBOT_KINEMATICS_HPP