#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from functools import partial
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle
 
class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.declare_parameter("spawn_frequency", 1.0)
        self.declare_parameter("turtle_name_prefix", "turtle")

        #self.turtle_name_prefix_ = "turtle"
        self.spawn_frequency = self.get_parameter("spawn_frequency").value
        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
        self.turtle_counter_ = 0
        self.alive_turtles_ = []
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray,
                                                              "alive_turtles", 10)
        self.spawn_turtle_timer_ = self.create_timer(1.0/self.spawn_frequency, self.spawn_new_turtle)
        self.catch_turtle_service_ = self.create_service(CatchTurtle,
                                                         "catch_turtle",
                                                         self.callback_catch_turtle)
        self.get_logger().info("Turtle Spawner node has started...")

    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    def spawn_new_turtle(self):
        self.turtle_counter_ += 1
        name = self.turtle_name_prefix_ + str(self.turtle_counter_)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.call_spawn_server(name, x, y, theta)

    def call_spawn_server(self, turtle_name, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Spawn...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_spawn,
            turtle_name=turtle_name, x=x, y=y, theta=theta))

    def callback_call_spawn(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info("Turtle " + response.name + " is now alive.")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
 
    def call_kill_server(self, turtle_name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server Kill...")

        request = Kill.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_kill,
            turtle_name=turtle_name))

    def callback_call_kill(self, future, turtle_name):
        try:
            response = future.result()
            # We need the index, so we use enumerate
            for (i, turtle) in enumerate(self.alive_turtles_):
                if turtle.name == turtle_name:
                    del self.alive_turtles_[i]
                    self.publish_alive_turtles()
                    break
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
 
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == "__main__":
    main()

