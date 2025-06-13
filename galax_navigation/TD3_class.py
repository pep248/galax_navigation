#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
import os
import rclpy
from rclpy.node import Node
import numpy as np
import onnxruntime as ort
from custom_interfaces.msg import Observations  # example observation msg
from custom_interfaces.msg import Dwa
from std_msgs.msg import Bool  # example goal reached msg


from sensor_msgs.msg import JointState          # example state msg
from std_msgs.msg import Float32MultiArray      # example action msg

class MyTD3Class(Node):
    
    def __init__(self):
        super().__init__('td3_policy_node')
        
        package_share = get_package_share_directory('galax_navigation')
        onnx_path = os.path.join(package_share, 'agent', 'td3_actor.onnx')
        self.session = ort.InferenceSession(
            onnx_path,
            providers=['CPUExecutionProvider']
        )

        self.obs_key = self.session.get_inputs()[0].name

        # --- ROS I/O ---
        self.latest_obs = None
        self.observations_subs = self.create_subscription(
            Observations,
            '/normalized_observations',
            self.obsCallback,
            1
        )
        
        self.goal_reached_subs = self.create_subscription(
            Bool,
            '/goal_reached',
            self.goalReachedCallback,
            1
        )
        self.goal_reached = False
        
        self.pub_dwa = self.create_publisher(Dwa, '/dwa_parameters', 1)
        
        self.timer = self.create_timer(0.1, self.timerCallback)  # 0.1 seconds = 100 ms

    def obsCallback(self, msg):
        self.latest_obs = msg
        # self.get_logger().info(f"Received normalized observations, distance_goal: {msg.distance_goal}")

    def goalReachedCallback(self, msg: Bool):
        if (msg.data):
            self.get_logger().info("Goal reached!")
            self.goal_reached = True
        else:
            self.get_logger().info("New goal received, resetting goal reached status.")
            self.goal_reached = False
        
    def timerCallback(self):
        if self.latest_obs is None:
            self.get_logger().warn("No observation received yet.")
            return

        # Convert Observations msg to numpy array (adjust field order as needed)
        obs = self.latest_obs
        obs_vec = np.array([
            obs.distance_goal,
            obs.distance_next_marker,
            obs.relative_direction_next_marker,
            obs.linear_velocity,
            obs.angular_velocity,
            *obs.closest_distance_sector  # unpack the list
        ], dtype=np.float32).reshape(1, -1)  # shape: (1, obs_dim)

        # Run ONNX inference
        action = self.session.run(None, {self.obs_key: obs_vec})[0]  # shape: (1, 4) if 4 outputs

        # Publish as Dwa message (assuming order: alpha, beta, gamma, delta)
        if not self.goal_reached:
            msg = Dwa()
            msg.alpha = float(action[0][0])
            msg.beta  = float(action[0][1])
            msg.gamma = float(action[0][2])
            msg.delta = float(action[0][3])
            self.pub_dwa.publish(msg)
            self.get_logger().info(f"Published DWA: {msg}")