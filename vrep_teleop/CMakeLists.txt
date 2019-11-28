cmake_minimum_required(VERSION 2.8.3)
project(vrep_teleop)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES diff_joy_control
#  CATKIN_DEPENDS roscpp geometry_msgs joy
#  DEPENDS
)

###########
## Build ##
###########

add_executable(diff_key_control src/diff_key_control.cpp)
add_executable(diff_key_publish src/diff_key_publish.cpp)
target_link_libraries(diff_key_control ${catkin_LIBRARIES})
target_link_libraries(diff_key_publish ${catkin_LIBRARIES})

