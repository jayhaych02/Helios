/**
 * @brief Abstract Class that defines properties for all Robots
 * @details PURE virtual functions("=0") - MUST be implemented by derived classes
 * @details virtual void implementations - CAN be overridden by derived classes
 * @details Non-virtual functions - common to all robots
 */
#pragma once
#include "common.hpp"


enum class ROBOT_STATE {
    IDLE,
    MOVING,
    EXECUTING_TASK,
    ERROR,
    EMERGENCY
};

class Robot_Base : public rclcpp::Node {
protected:
   
    /**
     * @brief Core Sensors
     * @link https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/
     */
    LidarSubscriberTwoDimensions sub_lidar;
    ImageSubscriber sub_depth_camera;
    Float64Subscriber sub_gas_sensor;
    ImageSubscriber sub_thermal_camera;
    
    /**
     * @brief Odometry and localization variables
     * @var pub_raw_odom Publish Odom msg to be used for Nav2
     * @var odom_sub Subscribes to Sensor Fused Odometry msg
     */
    OdomPublisher pub_raw_odom; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    
    // Robot state tracking
    ROBOT_ATTRIBUTES_t robot_attributes;
    ROBOT_STATE robot_state = ROBOT_STATE::IDLE;
    double battery_level = 100.0;
    geometry_msgs::msg::PoseStamped current_pose;
    
    StringPublisher pub_status; // Communication
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr navigate_client;// Navigation
    std::string current_environment = "unknown"; // indoor, outdoor, mixed  -->  Environment detection

public:
    Robot_Base(const std::string& name, const ROBOT_ATTRIBUTES_t& attributes)
    : Node(name),
      robot_attributes(attributes) {
        
        pub_status = this->create_publisher<std_msgs::msg::String>("/helios/robots/" + name + "/status", 10);
        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>("/helios/odom", 10,std::bind(&Robot_Base::odomCallback, this, std::placeholders::_1));    
        navigate_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    }
    
    virtual ~Robot_Base() = default;
    
    // Core functions all robots must implement
    virtual void initialize() = 0;
    virtual void updateSensors() = 0;
    virtual bool executeTask(const std::string& task_type, const geometry_msgs::msg::PoseStamped& location) = 0;
    
    // Status reporting
    virtual void reportStatus(const std::string& status) {
        auto msg = std_msgs::msg::String();
        msg.data = robot_attributes.robot_id + ":" + robot_attributes.robot_type + ":" + status + ":" + current_environment;
        pub_status->publish(msg);
    }

    /**
     * @brief Accepts fused Odometry Msg for use in Nav2 Stack
    */
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pose.header = msg->header;
        current_pose.pose = msg->pose.pose;
        
        // Update robot attributes with current position
        robot_attributes.x_pos = current_pose.pose.position.x;
        robot_attributes.y_pos = current_pose.pose.position.y;
        
        // Extract theta from quaternion
        tf2::Quaternion q(
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        robot_attributes.theta = yaw;
        robot_attributes.robot_pose = {robot_attributes.x_pos, robot_attributes.y_pos, robot_attributes.theta};
    }
    
    // Navigation helper
    void navigateTo(const geometry_msgs::msg::PoseStamped& target) {
        if (!navigate_client->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Navigation action server not available");
            return;
        }
        
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose = target;
        
        robot_state = ROBOT_STATE::MOVING;
        reportStatus("NAVIGATING");
        
        auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback =std::bind(&Robot_Base::goalResponseCallback, this, std::placeholders::_1);
        send_goal_options.result_callback = std::bind(&Robot_Base::resultCallback, this, std::placeholders::_1);
            
        navigate_client->async_send_goal(goal, send_goal_options);
    }
    
    // Navigation callbacks
    void goalResponseCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr& goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            robot_state = ROBOT_STATE::IDLE;
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server");
        }
    }
    
    void resultCallback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Navigation successful");
                robot_state = ROBOT_STATE::IDLE;
                reportStatus("NAVIGATION_COMPLETE");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                robot_state = ROBOT_STATE::ERROR;
                reportStatus("NAVIGATION_ABORTED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_INFO(this->get_logger(), "Goal was canceled");
                robot_state = ROBOT_STATE::IDLE;
                reportStatus("NAVIGATION_CANCELED");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                robot_state = ROBOT_STATE::ERROR;
                reportStatus("NAVIGATION_ERROR");
                break;
        }
    }
    
    // Environment detection
    virtual void detectEnvironment() {
        // Default implementation - derived classes can override with sensor-based detection
        // Could use LIDAR feature detection, GPS signal strength, etc.
        current_environment = "unknown";
    }
};