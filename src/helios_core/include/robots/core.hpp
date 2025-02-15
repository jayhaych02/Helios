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
    double battery_level = 100.0;  // Battery percentage
}Robot_Attributes_t;

using StringPublisher = rclcpp::Publisher<std_msgs::msg::String>::SharedPtr;
using TwistPublisher = rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr;
using LidarSubscriberTwoDimensions = rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr;

/* 
1.) ROS2 msg PointCloud2 represents 3D Lidar apparently
2.) Use OpenCV 

*/