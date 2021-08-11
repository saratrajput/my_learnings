#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
 
 
def main(args=None):
    rclpy.init(args=args)
    node = Node("add_two_ints_no_oop")
    
    client = node.create_client(AddTwoInts, "add_two_ints")
    # Wait for server till it is up
    while not client.wait_for_service(1.0):
        node.get_logger().warn("Waiting for Server Add Two Ints...")

    request = AddTwoInts.Request()
    request.a = int(input("Enter a: "))
    request.b = int(input("Enter b: "))

    # call is a synchronous call which will block until the response is given
    # Not recommened by ROS team as it can wait forever
    # call_async is recommended
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)

    try:
        response = future.result()
        node.get_logger().info(str(request.a) + " + " + str(request.b) + " = " +
                str(response.sum))
    except Exception as e:
        nodel.get_logger().error("Service call failed %r" % (e,))

    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()

