cd ../..
#catkin_make clean
source devel/setup.bash
catkin_make install --pkg wam_srvs
catkin_make install --pkg wam_msgs
catkin_make install
