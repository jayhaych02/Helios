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
#include <geometry_msgs/msg/twist.hpp>

/**
 * @brief Robot_Base class required variables that child classes must define/use
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

using StringPublisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
using StringSubscriber = rclcpp::Subscription<std_msgs::msg::String>::SharedPtr;
using TwistPublisher = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using TwistSubscriber = rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr;
using LidarSubscriberTwoDimensions = rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr;
using ImageSubscriber = rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr;
using Float64Subscriber = rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr;
using OdomPublisher = rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr;
/* 
1.) ROS2 msg PointCloud2 represents 3D Lidar apparently
2.) Use OpenCV 
*/