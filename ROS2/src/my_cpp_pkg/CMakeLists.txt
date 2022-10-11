cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(my_robot_interfaces REQUIRED)

# Executable should not be named same as the name of the file or the name of the node
add_executable(cpp_node src/my_first_node.cpp)
# Link dependencies specific to ament
ament_target_dependencies(cpp_node rclcpp)

# Add executable to robot news station node
add_executable(robot_news_station src/robot_news_station.cpp)
ament_target_dependencies(robot_news_station rclcpp example_interfaces)

# Executable for smartphone node
add_executable(smartphone src/smartphone.cpp)
ament_target_dependencies(smartphone rclcpp example_interfaces)

# Executable for number publisher node
add_executable(number_publisher src/number_publisher.cpp)
ament_target_dependencies(number_publisher rclcpp example_interfaces)

# Executable for number counter node
add_executable(number_counter src/number_counter.cpp)
ament_target_dependencies(number_counter rclcpp example_interfaces)

# Executable for add two ints server
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
ament_target_dependencies(add_two_ints_server rclcpp example_interfaces)

# Executable for add two ints client with no oop
add_executable(add_two_ints_client_no_oop src/add_two_ints_client_no_oop.cpp)
ament_target_dependencies(add_two_ints_client_no_oop rclcpp example_interfaces)

# Executable for add two ints client with oop
add_executable(add_two_ints_client src/add_two_ints_client.cpp)
ament_target_dependencies(add_two_ints_client rclcpp example_interfaces)

# Executable for hardware status publisher
add_executable(hw_status_publisher src/hw_status_publisher.cpp)
ament_target_dependencies(hw_status_publisher rclcpp my_robot_interfaces)

# Executable for led status publisher
add_executable(led_panel src/led_panel.cpp)
ament_target_dependencies(led_panel rclcpp my_robot_interfaces)

install(TARGETS
    cpp_node
    robot_news_station
    smartphone
    number_publisher
    number_counter
    add_two_ints_server
    add_two_ints_client_no_oop
    add_two_ints_client
    hw_status_publisher
    led_panel
    DESTINATION lib/${PROJECT_NAME} # Project name same as the name of th package
)

ament_package()
