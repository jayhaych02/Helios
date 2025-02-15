/**
 * @brief Abstract Class that defines properties for all Robots
 * @details virtual void functions - Must be implemented by derived classes
 * @details virtual void implementations - CAN be overridden by derived classes
 * @details Non-virtual functions - common to all robots

 */
#pragma once
#include "core.hpp"

class Robot_Base : public rclcpp::Node{
    
protected:
    StringPublisher pub_status;
    TwistPublisher pub_cmd_vel;
    LidarSubscriberTwoDimensions sub_lidar;

public:
    virtual void initialize() = 0;                     
    virtual void executeTask() = 0;                    
    virtual void handleEmergency() = 0;               
    virtual void processEnvironmentData() = 0;        
    virtual bool checkTaskCompletion() = 0;          

    virtual void move(const geometry_msgs::msg::Twist& cmd) {
        pub_cmd_vel->publish(cmd);
    }

    //if the robot received a msg to stop operations, cease all movement
    virtual void stop(const std::string& stop_msg) {
        if( strcmp(stop_msg,"stop operations") == 0 )
        auto stop_cmd = geometry_msgs::msg::Twist();
        stop_cmd.linear.x = 0;
        stop_cmd.linear.y = 0;
        stop_cmd.linear.z = 0;
        stop_cmd.angular.x = 0;
        stop_cmd.angular.y = 0;
        stop_cmd.angular.z = 0;
        pub_cmd_vel->publish(stop_cmd);
    }

    void reportStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        pub_status->publish(msg);
    }

};