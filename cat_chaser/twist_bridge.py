#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistBridge(Node):
    def __init__(self):
        super().__init__('twist_bridge')
        
        self.subscription = self.create_subscription(
            TwistStamped,
            '/cmd_vel_smoothed',
            self.twist_stamped_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.get_logger().info('Twist bridge started: TwistStamped -> Twist')

    def twist_stamped_callback(self, stamped_msg):
        twist_msg = Twist()
        twist_msg.linear = stamped_msg.twist.linear
        twist_msg.angular = stamped_msg.twist.angular
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    bridge = TwistBridge()
    rclpy.spin(bridge)

if __name__ == '__main__':
    main()