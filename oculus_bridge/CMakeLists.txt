cmake_minimum_required(VERSION 3.0.2)
project(oculus_bridge)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS

  cv_bridge
  image_transport
  roscpp
  roslib
  rospy
  std_msgs
  rosbag
  sensor_msgs
)
find_package(OpenCV)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES oculus_bridge
   CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(node_joybridge src/JoyBridge_Node.cpp src/JoyBridge_Class.cpp )
add_dependencies(node_joybridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_joybridge ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(node_videostream src/VideoStream_Node.cpp src/VideoStream_Class.cpp src/udp_interface/udp_client.cpp)
add_dependencies(node_videostream ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_videostream ${catkin_LIBRARIES})

add_executable(node_simvideostream src/SimVisionBridge_Class.cpp src/SimVisionBridge_Node.cpp src/udp_interface/udp_client.cpp)
add_dependencies(node_simvideostream ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_simvideostream ${catkin_LIBRARIES})