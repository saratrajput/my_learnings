cmake_minimum_required(VERSION 3.0.2)
project(ros1_app_py)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  my_robot_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)