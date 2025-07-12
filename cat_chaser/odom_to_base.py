#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class OdomToTF(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        
        # Create subscription to odometry topic
        self.subscription = self.create_subscription(
            Odometry,
            '/model/hiwonder/odometry', 
            self.odom_callback,
            10)
        
        # Create TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info('Odom to TF node started')
        
    def odom_callback(self, msg):
        # Create transform message
        t = TransformStamped()
        
        # Set header
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        
        # Set translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        
        # Set rotation
        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()