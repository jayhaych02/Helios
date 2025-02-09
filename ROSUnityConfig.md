# ROS2-Unity Integration Setup

This file contains the setup and configuration for integrating ROS2 (Humble) with Unity for robotics simulation and visualization.

## Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- Unity 6 Editor (Unity 6 is the most recent version)
- WSL (Windows Subsystem for Linux) 

## Installation

### 1. ROS TCP Endpoint Setup

1. Navigate to "/robotics/Helios/src " workspace source folder and clone the ROS-TCP-Endpoint repository:
```bash
cd ~/your_workspace/src
git clone -b main-ros2 https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git
```

2. Build the workspace:
```bash
cd ~/your_workspace
colcon build
source install/setup.bash
colcon build
source install/setup.bash
```

### 2. Unity Project Setup

1. Create a new Unity project
   - Select "High Definition 3D" under Core templates
   - This template is optimized for high-fidelity 3D rendering and real-time visualization

2. Install ROS TCP Connector
   - Open Package Manager while inside of that new project
   - Add package from git URL:
   ```
   https://github.com/Unity-Technologies/ROS-TCP-Connector.git?path=/com.unity.robotics.ros-tcp-connector
   ```

3. Configure ROS Settings
   - Navigate to ROS Settings in the Unity menu bar
   - Set Protocol to "ROS2"
   - Set IP address to your WSL IP (obtain using `hostname -I` command. Note: "sudo apt install net-tools" installs this command)

4. Project Structure
   - Create a "Scripts" folder in Assets
   - Add all C# scripts that interact with ROS2 inside this folder
   - Attach scripts to relevant game objects

## Running the Integration

### 1. Start ROS TCP Endpoint
```bash
ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=<your_ip_address>
```

### 2. Verify Endpoint
In a new terminal:
```bash
ros2 node list
```
You should see `/UnityTCPEndpoint` in the list of active nodes.

### 3. Example ROS2 Node

Below is an example ROS2 publisher node:

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TestUnityIntegration : public rclcpp::Node {
public:
     TestUnityIntegration()
     : Node("PubToUnity")
    {     
        publisher = create_publisher<std_msgs::msg::String>("test_topic", 10);
        timer = create_wall_timer(std::chrono::seconds(1), 
                                std::bind(&TestUnityIntegration::publish, this));
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
```

## Workflow

1. Run your setup shell script (if applicable)
2. Terminal 1: Run ROS TCP Endpoint
3. Terminal 2: Launch your ROS2 node
4. Start Unity simulation (Press Play)
5. Monitor Unity Console for messages

## Troubleshooting

- If Unity cannot connect to ROS, verify:
  - IP address configuration in ROS Settings
  - TCP Endpoint is running
  - Firewall settings are compatible
  - All required ROS2 packages are built and sourced

## References

- [Unity-Robotics-Hub Documentation](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/main/tutorials/ros_unity_integration/setup.md)
- [ROS-TCP-Connector Repository](https://github.com/Unity-Technologies/ROS-TCP-Connector)