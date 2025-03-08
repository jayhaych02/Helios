#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include <cmath>
#include <array>
#include <vector>
#include "sensor_msgs/msg/joint_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_kinematics.hpp"

using namespace helios_kinematics;

class KinematicsTest : public ::testing::Test {
protected:
    void SetUp() override {
        rclcpp::init(0, nullptr);
    }

    void TearDown() override {
        rclcpp::shutdown();
    }
};

// Test calculate_wheel_change function
TEST_F(KinematicsTest, TestCalculateWheelChange) {
    // Set up test inputs
    auto js_prev = sensor_msgs::msg::JointState();
    js_prev.header.stamp.sec = 0;
    js_prev.header.stamp.nanosec = 0;
    js_prev.position = {0.0, 0.0};
    
    auto js_new = sensor_msgs::msg::JointState();
    js_new.header.stamp.sec = 1;
    js_new.header.stamp.nanosec = 0;
    js_new.position = {0.5, -0.75};
    
    // Desired outputs
    double delta_wheel_l_true = 0.5;
    double delta_wheel_r_true = -0.75;
    double delta_t_true = 1.0;
    
    // Calculated outputs
    auto data = calculate_wheel_change(js_new, js_prev);
    
    // Ensure that calculated outputs match desired outputs
    EXPECT_DOUBLE_EQ(delta_wheel_l_true, data[0]) << "Data wheel left has the wrong value";
    EXPECT_DOUBLE_EQ(delta_wheel_r_true, data[1]) << "Data wheel right has the wrong value";
    EXPECT_DOUBLE_EQ(delta_t_true, data[2]) << "Data time has the wrong value";
}

// Test calculate_displacement function
TEST_F(KinematicsTest, TestCalculateDisplacement) {
    // Set up test inputs
    std::vector<double> delta_wheel_l = {0.696, -0.447, 0.676, 0.38, -0.4};
    std::vector<double> delta_wheel_r = {0.325, 0.083, -0.711, 0.629, 0.378};
    std::vector<double> wheel_radius = {0.113, 0.7, 0.585, 0.26, 0.231};
    std::vector<double> wheel_separation = {0.191, 0.331, 0.293, 0.559, 0.77};
    
    // Desired outputs
    std::vector<double> delta_s_true = {0.0576865, -0.1274, -0.0102375, 0.13117, -0.002541};
    std::vector<double> delta_theta_true = {-0.21949215, 1.12084592, -2.76926621, 0.11581395, 0.2334};
    
    for (size_t i = 0; i < delta_wheel_l.size(); i++) {
        // Calculated outputs
        auto result = calculate_displacement(
            delta_wheel_l[i], delta_wheel_r[i], wheel_radius[i], wheel_separation[i]);
        
        double delta_s = result[0];
        double delta_theta = result[1];
        
        // Ensure that calculated outputs match desired outputs
        EXPECT_NEAR(delta_s_true[i], delta_s, 1e-6) 
            << "Linear displacement is wrong for test case " << i;
        EXPECT_NEAR(delta_theta_true[i], delta_theta, 1e-6) 
            << "Angular displacement is wrong for test case " << i;
    }
}

// Test calculate_pose function
TEST_F(KinematicsTest, TestCalculatePose) {
    // Set up test inputs
    std::array<double, 3> pose_in = {0.5, 0.3, -1.0};
    double delta_s = 0.2;
    double delta_theta = 0.1;
    
    // Desired outputs
    std::array<double, 3> pose_true = {0.6163366178927767, 0.13731689904212524, -0.9};
    
    // Calculated outputs
    auto pose = calculate_pose(pose_in, delta_s, delta_theta);
    
    // Ensure that calculated outputs match desired outputs
    EXPECT_NEAR(pose_true[0], pose[0], 1e-6) << "Pose x is wrong";
    EXPECT_NEAR(pose_true[1], pose[1], 1e-6) << "Pose y is wrong";
    EXPECT_NEAR(pose_true[2], pose[2], 1e-6) << "Pose theta is wrong";
}

// Test calculate_wheel_change with NaN values
TEST_F(KinematicsTest, TestCalculateWheelChangeWithNaN) {
    // Set up test inputs
    auto js_prev = sensor_msgs::msg::JointState();
    js_prev.header.stamp.sec = 0;
    js_prev.header.stamp.nanosec = 0;
    js_prev.position = {0.0, 0.0};
    
    auto js_new = sensor_msgs::msg::JointState();
    js_new.header.stamp.sec = 1;
    js_new.header.stamp.nanosec = 0;
    js_new.position = {std::numeric_limits<double>::quiet_NaN(), -0.75};
    
    // Calculated outputs
    auto data = calculate_wheel_change(js_new, js_prev);
    
    // Ensure NaN values are handled correctly
    EXPECT_DOUBLE_EQ(0.0, data[0]) << "NaN wheel left value not handled correctly";
    EXPECT_DOUBLE_EQ(-0.75, data[1]) << "Data wheel right has the wrong value";
    EXPECT_DOUBLE_EQ(1.0, data[2]) << "Data time has the wrong value";
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}