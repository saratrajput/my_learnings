#!/usr/bin/env python3

import rclpy
from rclpy.node import Node # Import Node Class

# Create a class inheriting from the "Node" object
class MyNode(Node):
    
    def __init__(self):
        # Call the init function from "Node" object
        super().__init__("py_test")

        self.counter_ = 0
        self.get_logger().info("Hello ROS2")

        # Call a function at a given frequency
        self.create_timer(0.5, self.timer_callback) # This will run at 2 Hz

    def timer_callback(self):
        self.counter_ += 1
        self.get_logger().info("Hello " + str(self.counter_))



def main(args=None):
    # Initialize ROS2 communication
    # First line necessary for all scripts
    rclpy.init(args=args)

    # Create node
    #node = Node("py_test") # Don't name it same as name of the file
    node = MyNode()

    # Print something
    #node.get_logger().info("Hello ROS2")

    # Spin
    rclpy.spin(node)

    # Shutdown ROS2 communication
    rclpy.shutdown()


if __name__ == "__main__":
    main()
