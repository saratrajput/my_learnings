from launch import LaunchDescription
from launch_ros.actions import Node

# The name has to be exactly this because when you install the launch file the
# launch functionality will create a new program which will look for a function
# with this name to launch the application
def generate_launch_description(): 
    ld = LaunchDescription()

    # You can create a remap rule and apply it to any or all nodes
    remap_number_topic = ("number", "my_number")

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

    counter_node = Node(
            package="my_cpp_pkg",
            executable="number_counter",
            name="my_number_counter",
            remappings=[
                ("number_count", "my_number_count")
            ]
    )

    ld.add_action(number_publisher_node)
    ld.add_action(counter_node)
    return ld
