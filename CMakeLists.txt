cmake_minimum_required(VERSION 2.8.3)
project(robot_motion)

find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(FILES Num.msg)

add_service_files(FILES AddTwoInts.srv)


generate_messages(DEPENDENCIES std_msgs)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################


###################################
## catkin specific configuration ##
###################################
catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
)

## add include files
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(
  ${catkin_INCLUDE_DIRS}
)
include_directories(${OpenCV_INCLUDE_DIRS})

###########
## Build ##
###########

## Declare a C++ executable
add_executable(robot_motion_node src/robot_motion_node.cpp)
target_link_libraries(robot_motion_node ${catkin_LIBRARIES})
add_dependencies(robot_motion_node robot_motion_generate_messages_cpp)

add_executable(publisher_node src/publisher.cpp)
target_link_libraries(publisher_node ${catkin_LIBRARIES})
add_dependencies(publisher_node robot_motion_generate_messages_cpp)
#############
## Install ##
#############

install(DIRECTORY filter launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

