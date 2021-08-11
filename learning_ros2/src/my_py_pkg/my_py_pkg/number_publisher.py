#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
 
 
class NumberPublisherNode(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.declare_parameter("number_to_publish", 2)
        self.declare_parameter("publish_frequency", 1)

        self.number_ = self.get_parameter("number_to_publish").value
        # Create publisher
        self.publisher_ = self.create_publisher(Int64, "number", 10)
        # Start timer and publish in the given frequency
        self.publish_frequency_ = self.get_parameter("publish_frequency").value
        self.timer = self.create_timer(1 / self.publish_frequency_, self.publish_number)
        # Log that the publisher has started
        self.get_logger().info("Robot News Station has been started.")

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)
 
 
def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
