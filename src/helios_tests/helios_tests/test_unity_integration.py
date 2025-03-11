#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TestUnityIntegration(Node):
    """Node for testing integration with Unity."""
    
    def __init__(self):
        super().__init__('PubToUnity')
        self.publisher = self.create_publisher(String, 'test_topic', 10)
        self.timer = self.create_timer(1.0, self.publish)
    
    def publish(self):
        """Publish test message."""
        msg = String()
        msg.data = 'Test from ROS2'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = TestUnityIntegration()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()