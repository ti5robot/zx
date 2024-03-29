cmake_minimum_required(VERSION 3.0.2)
project(Ti5robot)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
  serial
)


catkin_package(
  INCLUDE_DIRS include  
LIBRARIES Ti5robot
)


include_directories(
include
${catkin_INCLUDE_DIRS}
)

add_library(Ti5robot
	src/Ti5robot.cpp
)

#add_dependencies(Ti5robot
#	${${PROJECT_NAME}_EXPORTED_TARGETS}
#	${catkin_EXPORTED_TARGETS}
#)

target_link_libraries(Ti5robot
${catkin_LIBRARIES}
/home/ti5robot/controlcan/libcontrolcan.so
pthread
)


