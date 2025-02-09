#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TestUnityIntegration : public rclcpp::Node {
public:
     TestUnityIntegration()
     : Node("PubToUnity")
    {     
        publisher = create_publisher<std_msgs::msg::String>("test_topic", 10);
        timer = create_wall_timer(std::chrono::seconds(1), std::bind(&TestUnityIntegration::publish, this));
    }
 
private:

    void publish() {
        auto msg = std_msgs::msg::String();
        msg.data = "Test from ROS2";
        publisher->publish(msg);
    }
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;   

};

 int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestUnityIntegration>());
  rclcpp::shutdown();
  return 0;
}

