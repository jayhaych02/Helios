/**
 * @brief Abstract Class that defines properties for all Robots
 * @details PURE virtual functions("=0") - MUST be implemented by derived classes
 * @details virtual void implementations - CAN be overridden by derived classes
 * @details Non-virtual functions - common to all robots
 * @todo Implement specific robot classes to determine what's needed in this class as well
 */
#pragma once
#include "core.hpp"

class Robot_Base : public rclcpp::Node{
    
protected:

    ROBOT_ATTRIBUTES_t robot_attributes;
    StringPublisher broadcast_status;
    LidarSubscriberTwoDimensions sub_lidar;
    OdomPublisher pub_raw_odom;
    ImageSubscriber sub_depth_camera;
    Float64Subscriber sub_gas_sensor;
    ImageSubscriber sub_thermal_camera;

    
public:
    virtual void initialize() = 0;                     
    virtual void executeTask() = 0;                    
    virtual void handleEmergency() = 0;               
    virtual void processEnvironmentData() = 0;        
    virtual bool checkTaskCompletion() = 0;          
    

    /**
     * @brief if the robot received a msg to stop operations from the Central Control Server, cease all movement
     */
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
    /**
     * @brief Each robot should report its status as a string to Central Control Server
     */
    virtual void reportStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        pub_status->publish(msg);
    }

};