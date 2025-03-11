#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PointStamped
import tf2_geometry_msgs


class RobotTfListener(Node):
    def __init__(self):
        super().__init__('robot_tf_listener')
        
        self.tf_buffer = tf2_ros.Buffer(node=self)
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.transform_point)
        
    def transform_point(self):
        # Create a point in the base_laser frame that we'd like to transform to the base_link frame
        laser_point = PointStamped()
        laser_point.header.frame_id = "base_laser"
        laser_point.header.stamp = self.get_clock().now().to_msg()
        laser_point.point.x = 1.0  # arbitrary point in space
        laser_point.point.y = 0.2
        laser_point.point.z = 0.0
        
        try:
            # Wait for the transform to be available
            if not self.tf_buffer.can_transform(
                    "base_link", "base_laser", rclpy.time.Time()):
                self.get_logger().info("Waiting for transform...")
                return
                
            # Transform the point
            base_point = self.tf_buffer.transform(
                laser_point, "base_link")
                
            self.get_logger().info(
                f"base_laser: ({laser_point.point.x:.2f}, {laser_point.point.y:.2f}, {laser_point.point.z:.2f}) "
                f"-----> base_link: ({base_point.point.x:.2f}, {base_point.point.y:.2f}, {base_point.point.z:.2f}) "
                f"at time {self.get_clock().now().seconds_nanoseconds()[0]:.2f}")
                
        except TransformException as ex:
            self.get_logger().error(
                f"Received an exception trying to transform a point from \"base_laser\" to \"base_link\": {ex}")


def main(args=None):
    rclpy.init(args=args)
    node = RobotTfListener()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()