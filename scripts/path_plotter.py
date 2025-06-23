#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose
import tf2_ros
import tf_transformations
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class PathPlotter(Node):
    def __init__(self):
        super().__init__('path_plotter')

        # Path storage
        self.path = []
        self.recording = False

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers and subscribers
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.path_pub = self.create_publisher(Path, '/robot_movement', 10)
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, qos)
        self.goal_reached_sub = self.create_subscription(
            Bool, '/goal_reached', self.goal_reached_callback, qos)

        # Timer for periodic path update
        self.timer = self.create_timer(0.2, self.timer_callback)

        # Frame names
        self.map_frame = 'map'
        self.base_frame = 'galax_base_link'

    def goal_pose_callback(self, msg):
        self.get_logger().info('Received new goal pose, starting path recording.')
        self.path = []
        self.recording = True

    def goal_reached_callback(self, msg):
        if msg.data:
            self.get_logger().info('Goal reached, stopping path recording.')
            self.recording = False

    def timer_callback(self):
        # Try to get the current robot pose
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = self.map_frame
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            if self.recording:
                self.path.append(pose)

            # Publish the path
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = self.map_frame
            path_msg.poses = self.path
            self.path_pub.publish(path_msg)

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PathPlotter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()