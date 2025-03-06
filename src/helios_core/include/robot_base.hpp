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
    LidarSubscriberTwoDimensions sub_lidar;
    OdomPublisher pub_raw_odom;
    ImageSubscriber sub_depth_camera;    

    ROBOT_ATTRIBUTES_t robot_attributes;
    StringPublisher broadcast_status;
    
    Float64Subscriber sub_gas_sensor;
    ImageSubscriber sub_thermal_camera;

    /**
    * @brief Each robot should report its status so other robot's are aware
    */
   virtual void reportStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        pub_status->publish(msg);
    }

    


  
    

 

};