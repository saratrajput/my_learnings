cmake_minimum_required(VERSION 3.0.2)
project(ros1_app_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  my_robot_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(number_publisher src/number_publisher.cpp)
target_link_libraries(number_publisher ${catkin_LIBRARIES})

add_executable(number_counter src/number_counter.cpp)
add_dependencies(number_counter my_robot_msgs_generate_messages_cpp)
target_link_libraries(number_counter ${catkin_LIBRARIES})

add_executable(oop_number_counter src/oop_number_counter.cpp)
add_dependencies(oop_number_counter my_robot_msgs_generate_messages_cpp)
target_link_libraries(oop_number_counter ${catkin_LIBRARIES})