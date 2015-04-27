cmake_minimum_required(VERSION 2.8.3)
project(p3at)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

## Declare ROS messages and services

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include ./include)

add_executable(keyboard_control src/keyboard_control.cpp)
target_link_libraries(keyboard_control ${catkin_LIBRARIES})
add_dependencies(keyboard_control p3at_gencpp)

add_executable(safety src/safety.cpp)
target_link_libraries(safety ${catkin_LIBRARIES})
add_dependencies(safety p3at_gencpp)

add_executable(safety_navigation src/safety_navigation.cpp)
target_link_libraries(safety_navigation ${catkin_LIBRARIES})
add_dependencies(safety_navigation p3at_gencpp)
