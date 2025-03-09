#pragma once

#include <rclcpp/rclcpp.hpp>
#include <cstddef>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>
#include <string>
#include <cstring>
#include <iostream>
#include <cassert>
#include <functional>
#include <future>
#include <chrono>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Dense> 
#include <geometry_msgs/msg/twist.hpp>
#include <list>
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include "std_msgs/msg/bool.hpp"
#include "rcl_interfaces/msg/parameter_event.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"  
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include <std_srvs/srv/trigger.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include <cmath>
#include <random>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using StringPublisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
using StringSubscriber = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;
using TwistPublisher = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using TwistSubscriber = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
using LidarSubscriberTwoDimensions = rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr;
using ImageSubscriber = rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr;
using Float64Subscriber = rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr;
using OdomPublisher = rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr;



/**
 * @brief Robot_Base class required variables that child classes must define/use
 * @note This implementation is NOT set in stone whatsoever. Will code and determine if this is even necessary
*/
typedef struct{
    std::string robot_type;        // Type of robot (scout, firefighter, etc)
    std::string robot_id;          // Unique identifier 
    bool is_active = false;        // Robot operational status
    double x_pos;
    double y_pos;
    double theta;                  
    std::array<double> robot_pose = {x_pos , y_pos , theta}; 
}ROBOT_ATTRIBUTES_t;
