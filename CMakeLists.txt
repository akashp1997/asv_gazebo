cmake_minimum_required(VERSION 2.8.3)
project(asv_gazebo)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  gazebo_ros
  hector_gazebo_plugins
  gazebo_plugins
  asv_description
)

find_package(GAZEBO REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
	LIBRARIES ${PROJECT_NAME}
	CATKIN_DEPENDS roscpp gazebo_ros std_msgs geometry_msgs
)

include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(thruster_control SHARED src/gazebo_thruster_controller.cpp)

target_link_libraries(thruster_control ${GAZEBO_LIBRARIES})
