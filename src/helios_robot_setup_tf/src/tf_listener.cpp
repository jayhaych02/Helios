#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class RobotTfListener : public rclcpp::Node
{
public:
  RobotTfListener() : Node("robot_tf_listener")
  {
    tf_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
    timer = this->create_wall_timer(std::chrono::seconds(1),std::bind(&RobotTfListener::transform_point, this));
  }

private:
  void transform_point(){
    
    geometry_msgs::msg::PointStamped laser_point;       // Create a point in the base_laser frame that we'd like to transform to the base_link frame
    laser_point.header.frame_id = "base_laser";
    laser_point.header.stamp = this->get_clock()->now();
    laser_point.point.x = 1.0;                              //  arbitrary point in space
    laser_point.point.y = 0.2;
    laser_point.point.z = 0.0;
    
    try {
      // Wait for the transform to be available
      if (!tf_buffer->canTransform("base_link", "base_laser", rclcpp::Time(0))) {
        RCLCPP_INFO(this->get_logger(), "Waiting for transform...");
        return;
      }
      
      // Transform the point
      geometry_msgs::msg::PointStamped base_point;
      base_point = tf_buffer->transform(laser_point, "base_link");
      
      RCLCPP_INFO(this->get_logger(), 
        "base_laser: (%.2f, %.2f, %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z,
        rclcpp::Time(base_point.header.stamp).seconds());
    }
    catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(this->get_logger(),
        "Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s",
        ex.what());
    }
  }

  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotTfListener>());
  rclcpp::shutdown();
  return 0;
}