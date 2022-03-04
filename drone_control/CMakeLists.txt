cmake_minimum_required(VERSION 3.0.2)
project(drone_control)

find_package(catkin REQUIRED COMPONENTS
  dji_sdk
  roscpp
  geometry_msgs
  rospy
  std_msgs
)
find_package(DJIOSDK REQUIRED)


set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${DJIOSDK_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(node_dronecontrol
        src/DroneControl_Node.cpp)

target_link_libraries(node_dronecontrol
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(node_dronecontrol dji_sdk_generate_messages_cpp)

add_executable(node_SimSample
        src/Sample_Node.cpp)

target_link_libraries(node_SimSample
        ${catkin_LIBRARIES}
        ${DJIOSDK_LIBRARIES}
        )
add_dependencies(node_SimSample dji_sdk_generate_messages_cpp)