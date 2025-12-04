#!/usr/bin/env python3
"""Publish fake static odometry for testing lane detector."""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

class FakeOdomPublisher(Node):
    def __init__(self):
        super().__init__('fake_odom_publisher')
        self.pub = self.create_publisher(Odometry, '/car1/odom', 10)
        self.timer = self.create_timer(0.05, self.publish_odom)  # 20 Hz
        self.get_logger().info('Publishing fake odometry on /car1/odom')

    def publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        # Static position at origin, facing forward (0° yaw)
        msg.pose.pose.position = Point(x=0.0, y=0.0, z=0.0)
        msg.pose.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = FakeOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
