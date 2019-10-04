cmake_minimum_required(VERSION 2.8.3)
project(wam_teleop)

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
link_directories(${BARRETT_LIBRARY_DIRS})
add_definitions(${BARRETT_DEFINITIONS})
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

add_executable(wam_teleop src/wam_joystick_teleop.cpp)
target_link_libraries(wam_teleop ${catkin_LIBRARIES} ${BARRETT_LIBRARIES})
