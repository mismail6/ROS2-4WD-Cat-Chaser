# cat_chaser/odom_tf_broadcaster.py

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            '/model/hiwonder/odometry',  # Update if your actual odom topic is different
            self.handle_odom,
            10)

    def handle_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        # t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'         # hardcoded to 'odom'
        t.child_frame_id = 'base_link'     # hardcoded to 'base_link'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
