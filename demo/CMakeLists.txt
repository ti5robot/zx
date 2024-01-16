cmake_minimum_required(VERSION 3.0.2)
project(demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  serial
  Ti5robot
)

catkin_package(
  INCLUDE_DIRS include
#   /home/ti5robot/src/Ti5robot/include
#   LIBRARIES demo
  CATKIN_DEPENDS roscpp rospy Ti5robot
  #    DEPENDS system_lib
)



include_directories(
 include
   /home/ti5robot/src/Ti5robot/include
   ${catkin_INCLUDE_DIRS}
)

add_executable(demo src/demo.cpp 
#	/home/ti5robot/src/Ti5robot/src/Ti5robot.cpp
#	/home/ti5robot/src/Ti5robot/src/SingleCaninterface.cpp
#	/home/ti5robot/src/Ti5robot/src/main.cpp
#	/home/ti5robot/src/Ti5robot/include/Ti5robot.h
)

add_dependencies(demo 
		${${PROJECT_NAME}_EXPORTED_TARGETS}
		${catkin_EXPORTED_TARGETS}
)

target_link_libraries(demo 
	${catkin_LIBRARIES}
	/opt/ros/melodic/lib/libmoveit_move_group_interface.so
#		/home/ti5robot/controlcan/libcontrolcan.so
#	pthread
)
