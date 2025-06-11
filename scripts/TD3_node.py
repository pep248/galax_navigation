#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from galax_navigation.TD3_class import MyTD3Class
from rclpy.executors import MultiThreadedExecutor

def main(args=None):
    rclpy.init(args=args)

    # Create nodes
    td3_node = MyTD3Class()

    # Create executor and add nodes
    executor = MultiThreadedExecutor()
    executor.add_node(td3_node)

    try:
        # Run the nodes within the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        executor.shutdown()
        td3_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()