cmake_minimum_required(VERSION 3.0.2)
project(arm_invkinematics)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  kdl_parser
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  tf
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs  
)


catkin_package(

   CATKIN_DEPENDS
   roscpp
   rospy
   kdl_parser
   std_msgs
   sensor_msgs
   geometry_msgs
   message_runtime

)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(node_arminvkine src/ArmInvKine_Node.cpp src/ArmInvKine_Class.cpp )
target_link_libraries(node_arminvkine ${catkin_LIBRARIES})
add_dependencies(node_arminvkine ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(node_arminvrout src/ArmInvRout_Node.cpp src/ArmInvRout_Class.cpp)
target_link_libraries(node_arminvrout ${catkin_LIBRARIES})
add_dependencies(node_arminvrout ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
