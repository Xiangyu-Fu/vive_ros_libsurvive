cmake_minimum_required(VERSION 3.0.2)
project(libsurvive_pub)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  geometry_msgs 
  tf)

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
#generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(
	${catkin_INCLUDE_DIRS}
	include
  include/libsurvive
  include/cnmatrix/include
  include/redist  
	)

link_directories(~/repos/libsurvive/bin)

add_executable(${PROJECT_NAME}_node src/libsurvive_pub_node.cc)
target_link_libraries(${PROJECT_NAME}_node 
	${catkin_LIBRARIES}
	survive
	)
	
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
