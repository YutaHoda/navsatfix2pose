cmake_minimum_required(VERSION 2.8.3)
project(navsatfix2pose)

find_package(catkin REQUIRED COMPONENTS
  eagleye_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  eagleye_coordinate
)

catkin_package(
  CATKIN_DEPENDS
  eagleye_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  eagleye_coordinate
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(navsatfix2pose src/navsatfix2pose.cpp)
target_link_libraries(navsatfix2pose ${catkin_LIBRARIES} )
add_dependencies(navsatfix2pose ${catkin_EXPORTED_TARGETS})
