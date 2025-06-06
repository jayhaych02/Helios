#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TugbotController : public rclcpp::Node
{

public:
  TugbotController() : Node("tugbot_controller")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/model/tugbot/cmd_vel", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1),std::bind(&TugbotController::timer_callback, this));
  }

private:
  void timer_callback()
  {
   
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0.5;  
    message.angular.z = 0.5;
    publisher_->publish(message);

    RCLCPP_INFO(this->get_logger(), "Publishing: linear.x='%f', angular.z='%f'",message.linear.x, message.angular.z);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TugbotController>());
  rclcpp::shutdown();
  return 0;
}