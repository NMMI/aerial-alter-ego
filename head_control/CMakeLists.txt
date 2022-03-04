cmake_minimum_required(VERSION 3.0.2)
project(head_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  kdl_parser
  sensor_msgs
  geometry_msgs
  message_runtime
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(head_control src/head_manager.cpp)
target_link_libraries(head_control ${catkin_LIBRARIES})