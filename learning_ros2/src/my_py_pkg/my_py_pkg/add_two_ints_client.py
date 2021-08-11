#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
# Allows us to add more arguments to our callback
from functools import partial
 
 
class AddTwoIntsClientNode(Node):
    def __init__(self):
        super().__init__("add_two_ints_client")
        self.call_add_two_ints_server(6, 7)
        # You can call them mutliple times for here
        #self.call_add_two_ints_server(4, 3)

    ### Start of template
    def call_add_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Add Two Ints...")

        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        # This add done callback is part of the future method and has nothing to do with
        # ROS. Here we use it since in python we can't wait asynchronously without
        # multi-threaded operation
        future.add_done_callback(partial(self.callback_call_add_two_ints, a=a, b=b))

    def callback_call_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(str(a) + " + " + str(b) + " = " +
                    str(response.sum))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    ### End of template

 
def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClientNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()

