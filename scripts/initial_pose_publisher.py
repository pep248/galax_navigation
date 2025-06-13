#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        msg.pose.covariance = [0.0]*36
        # Publish a few times to ensure delivery
        for _ in range(20):
            pub.publish(msg)
            self.get_logger().info('Published initial pose to /initialpose')
            rclpy.spin_once(self, timeout_sec=0.1)
        rclpy.shutdown()

def main():
    rclpy.init()
    node = InitialPosePublisher()

if __name__ == '__main__':
    main()