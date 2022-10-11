cmake_minimum_required(VERSION 3.0.2)
project(my_robot_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ResetCounter.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()

catkin_package(
  CATKIN_DEPENDS message_runtime    
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
