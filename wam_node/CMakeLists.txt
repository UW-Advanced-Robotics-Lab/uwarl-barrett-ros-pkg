cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
rosbuild_find_ros_package(actionlib_msgs)
include(${actionlib_msgs_PACKAGE_PATH}/cmake/actionbuild.cmake)
genaction()

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_genmsg()
rosbuild_gensrv()

link_directories(/usr/xenomai/lib /usr/local/lib)
rosbuild_add_executable(wam_node src/wam_node.cpp)
include_directories(/usr/xenomai/include /usr/local/include /usr/local/include/eigen2 /usr/local/include/boost)
add_definitions(-D_GNU_SOURCE -D_REENTRANT -D__XENO__)
rosbuild_add_compile_flags(wam_node -Wall -pipe)
target_link_libraries(wam_node barrett native xenomai pthread rt gsl gslcblas m boost_signals boost_thread)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(wam_node thread)


