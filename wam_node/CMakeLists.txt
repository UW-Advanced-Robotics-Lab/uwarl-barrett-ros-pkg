cmake_minimum_required(VERSION 2.8.3)
project(wam_node)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  wam_msgs
  wam_srvs
)
find_package(Barrett REQUIRED)
include_directories(${BARRETT_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  wam_msgs
  wam_srvs
)

add_executable(wam_node src/wam_node.cpp)
target_link_libraries(wam_node ${catkin_LIBRARIES} ${BARRETT_LIBRARIES})
