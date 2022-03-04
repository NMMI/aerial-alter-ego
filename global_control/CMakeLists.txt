cmake_minimum_required(VERSION 3.0.2)
project(global_control)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

catkin_package(

)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

