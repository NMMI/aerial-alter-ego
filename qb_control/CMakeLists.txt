cmake_minimum_required(VERSION 3.0.2)
project(qb_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp roslib)


## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qb_ctrl
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(node_qbmanager src/QbManager_Node.cpp src/QbManager_Class.cpp src/qbmove_communications.cpp src/imuboard_communications.cpp)
add_dependencies(node_qbmanager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(node_qbmanager ${catkin_LIBRARIES})


