#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node):  # MODIFY NAME
    def __init__(self):
        super().__init__("node_name")  # MODIFY NAME
        self.get_logger().info("Node has been started.")


def main(args=None):
    rclpy.init(args=args)
    node = MyCustomNode()  # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
