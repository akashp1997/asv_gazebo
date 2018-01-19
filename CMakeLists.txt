cmake_minimum_required(VERSION 2.8.3)
project(asv_gazebo)
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  gazebo REQUIRED
)
include_directories(${GAZEBO_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(buoyancy SHARED src/buoyancy.cc)
target_link_libraries(buoyancy ${GAZEBO_LIBRARIES})
