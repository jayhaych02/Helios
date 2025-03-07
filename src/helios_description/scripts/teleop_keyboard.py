#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select
import threading

msg = """
Control Your Helios Robot!
---------------------------
Moving around:
   q    w    e
   a    s    d
   z    x    c

w/x : increase/decrease linear velocity (forward/backward)
a/d : increase/decrease angular velocity (left/right)
s : stop
q/e : curve left/right
z/c : rotate in place left/right

CTRL-C to quit
"""

moveBindings = {
    'w': (1, 0, 0),
    'e': (1, -1, 0),
    'a': (0, 1, 0),
    'd': (0, -1, 0),
    'q': (1, 1, 0),
    'x': (-1, 0, 0),
    'c': (0, -1, 0),
    'z': (0, 1, 0),
    's': (0, 0, 0),
}

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.settings = self.saveTerminalSettings()
        self.key = None
        
        # Create a thread for keyboard input
        self.keyboard_thread = threading.Thread(target=self.getKey)
        self.keyboard_thread.daemon = True
        self.keyboard_thread.start()
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # Default motion commands
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.speed = 0.2
        self.turn = 1.0
        
        self.print_instructions()
    
    def saveTerminalSettings(self):
        return termios.tcgetattr(sys.stdin)
    
    def restoreTerminalSettings(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def getKey(self):
        while True:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                self.key = sys.stdin.read(1)
            else:
                self.key = None
            
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def print_instructions(self):
        self.get_logger().info(msg)
    
    def timer_callback(self):
        if self.key is not None:
            if self.key == '\x03':  # CTRL-C
                self.get_logger().info("Shutting down")
                self.restoreTerminalSettings()
                rclpy.shutdown()
                return
            
            if self.key in moveBindings.keys():
                self.x = moveBindings[self.key][0] * self.speed
                self.th = moveBindings[self.key][1] * self.turn
                self.get_logger().info(f"Linear: {self.x}, Angular: {self.th}")
            
            # Reset key to avoid repeated commands
            self.key = None
        
        # Create and publish Twist message
        twist = Twist()
        twist.linear.x = self.x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.th
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        twist = Twist()
        teleop.publisher.publish(twist)
        teleop.restoreTerminalSettings()
        teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()