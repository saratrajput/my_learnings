from launch import LaunchDescription
from launch_ros.actions import Node

# The name has to be exactly this because when you install the launch file the
# launch functionality will create a new program which will look for a function
# with this name to launch the application
def generate_launch_description(): 
    ld = LaunchDescription()

    remap_number_topic = ("number", "my_number")

    # node_declaration
    number_publisher_node = Node(
            package="my_py_pkg",
            executable="number_publisher",
            name="my_number_publisher",
            remappings=[
                #("number", "my_number") # Needs 2 arguments
                remap_number_topic
            ],
            parameters=[
                {"number_to_publish": 4},
                {"publish_frequency": 5.0}
            ],
    )

    ld.add_action(number_publisher_node)
    return ld
