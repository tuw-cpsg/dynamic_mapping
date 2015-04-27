cmake_minimum_required(VERSION 2.8.3)
project(dynamic_mapping)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs nav_msgs genmsg message_generation tf)

## Declare ROS messages and services
add_message_files(
	FILES
	DynamicGrid.msg
)

add_service_files(
	FILES
	ExportMap.srv
	GetDynamicMap.srv 
)
## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs nav_msgs dynamic_mapping)

## Declare a catkin package
catkin_package(
	CATKIN_DEPENDS message_runtime std_msgs nav_msgs
)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(map_import src/map_import.cpp)
target_link_libraries(map_import  SDL SDL_image ${catkin_LIBRARIES})
add_dependencies(map_import dynamic_mapping_gencpp)

add_executable(map_export src/map_export.cpp)
target_link_libraries(map_export ${catkin_LIBRARIES})
add_dependencies(map_export dynamic_mapping_gencpp)

add_executable(main src/main.cpp)
target_link_libraries(main map_import SDL SDL_image yaml-cpp ${catkin_LIBRARIES})
add_dependencies(main dynamic_mapping_gencpp)

add_executable(transform_broadcaster src/transform_broadcaster.cpp)
target_link_libraries(transform_broadcaster ${catkin_LIBRARIES})
add_dependencies(transform_broadcaster dynamic_mapping_gencpp)

