cmake_minimum_required(VERSION 3.0.2)
project(test1)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  moveit_core
  moveit_ros_planning_interface
  roscpp
)


catkin_package(
  INCLUDE_DIRS
  include  
/home/ti5robot/src/test1/include
LIBRARIES
test1
CATKIN_DEPENDS
roscpp 
DEPENDS
system_lib
)


include_directories(
include
/home/ti5robot/src/test1/include
${catkin_INCLUDE_DIRS}
)

add_executable(test1
	/home/ti5robot/src/test1/src/test1.cpp
	/home/ti5robot/src/test1/src/SingleCaninterface.cpp
)

add_dependencies(test1
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(test1
${catkin_LIBRARIES}
/home/ti5robot/controlcan/libcontrolcan.so
pthread
)


