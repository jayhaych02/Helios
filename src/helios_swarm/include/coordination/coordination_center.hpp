/**
 * @brief Multi-Robot/Swarm Centralized Coordination Hub
 * @details 5 minutes 57 seconds https://www.youtube.com/watch?v=WIt0xXEdNfw&list=PLKhfRkcyFZpOJxAuy9y9_OJV4aVbBfT3E&index=68
 */

#pragma once
#include "helios_core/core.hpp"

class CoordinationCenter : public rclcpp::Node{
    
private:
    const std::string stop_msg = "stop operations"; // "attach" to stop topic? 
    StringPublisher pub_stop_msg;

public:
    pub_stop_msg = this->create_publisher<std_msgs::msg::String>("stop_topic", 10);

    //Research consensus/flocking/market_based coordination
};