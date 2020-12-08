#!/bin/bash

echo "$1 ROS commands!"

for i in `seq 1 $1`;
do
  echo "Iteration: $i"
  
  #rosservice call /wam/go_home
  rosservice call /wam/joint_move "joints:
  - 0.0
  - -2.0
  - -0.0
  - 3.0
  - 0.0
  - 0.0
  - 0.0"  
  sleep 1

  rosservice call /wam/joint_move "joints:
  - 0.64
  - -1.83
  - -0.49
  - 0.96
  - -0.59
  - 0.03
  - 0.21"

  sleep 1
done

rosservice call /wam/go_home
