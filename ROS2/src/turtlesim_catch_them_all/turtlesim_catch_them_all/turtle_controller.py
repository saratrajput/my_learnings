#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from functools import partial
# To subscribe to the pose
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
 
 
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.declare_parameter("catch_closest_turtle_first", True)

        self.turtle_to_catch_ = None
        #self.target_x = 4.0
        #self.target_y = 4.0
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value

        self.pose_ = None
        # Subscribe to turtle pose
        self.cmd_vel_publisher_ = self.create_publisher(
                Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
                Pose, "turtle1/pose",
                self.callback_turtle_pose, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray,
                                                                  "alive_turtles",
                                                                  self.callback_alive_turtles,
                                                                  10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
        self.get_logger().info("Turtle controller node has launched...")

    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt((dist_x * dist_x) + (dist_y * dist_y))
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                # Get the first turtle in the array
                self.turtle_to_catch_ = msg.turtles[0]

    def control_loop(self):
        # We don't don anything if we don't have the current pose of turtle
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return

        #dist_x = self.target_x - self.pose_.x
        #dist_y = self.target_y - self.pose_.y
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y

        distance = math.sqrt((dist_x * dist_x) + (dist_y * dist_y))

        # We need to send the linear and angular velocity to move the turtle
        msg = Twist()

        if distance > 0.5: # Target hasn't been reached
            # position: We use only a P controller
            msg.linear.x = 2 * distance # 2 is factor for the P-controller
            # orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            # Normalize the angle
            if diff > math.pi:
                diff -= 2 * math.pi
            elif diff < -math.pi:
                diff += 2 * math.pi
            msg.angular.z = 6 * diff
            # Here 6 is a factor for the P-controller
        else: # Target has been reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.cmd_vel_publisher_.publish(msg)

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Catch Turtle...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle,
            turtle_name=turtle_name))

    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle " + str(turtle_name) + " could not be caught")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
 
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()
