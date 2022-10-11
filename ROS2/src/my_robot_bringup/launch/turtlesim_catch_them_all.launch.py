from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description(): 
    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    turtlesim_node = Node(
            package="turtlesim",
            executable="turtlesim_node")

    turtlesim_controller_node = Node(
            package="turtlesim_catch_them_all",
            executable="turtlesim_controller")

    turtle_spawner_node = Node(
            package="turtlesim_catch_them_all",
            executable="turtle_spawner",
            parameters=[
                {"spawn_frequency": 1.5},
                {"turtle_name_prefix": "my_turtle"}])

    ld.add_action(turtlesim_node)
    ld.add_action(turtlesim_controller_node)
    ld.add_action(turtle_spawner_node)
    return ld
