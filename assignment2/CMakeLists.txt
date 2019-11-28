cmake_minimum_required(VERSION 2.8.3)
project(assignment2)
find_package(
	catkin REQUIRED
	roscpp
	roslib
	std_msgs
	geometry_msgs
)

find_package(orocos_kdl REQUIRED)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories (/usr/include/eigen3)


add_executable(assignment2 src/assignment2.cpp)

add_dependencies(assignment2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


target_link_libraries(assignment2 ${catkin_LIBRARIES} orocos-kdl)
