# barrett-ros-pkg
> ROS package for the Barrett WAM and related products.

## Overview
This is Barrett Technology's ROS repository wrapping Libbarrett's functionalities.  Libbarrett is a real-time controls library written in C++ that runs Barrett Technology's products, including the WAM Arm and the BH8-282 BarrettHand.
- The `wam_robot` stack is designed to be run on a WAM PC-104 or external control PC.
- The `wam_common` stack is designed as the interface to communicate with the 
functionality exposed by the wam_node.

### Pre-Requisites
#### On  Ubuntu 14.04:
- An installed version of [Libbarrett 1.3.0](https://git.barrett.com/software/libbarrett/blob/release/release-1.3.0/README.txt)
- [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) 

#### On  Ubuntu 18.04:
- An installed version of [Libbarrett 2.0.0](https://git.barrett.com/software/libbarrett/blob/devel/README.md)
-  [ROS Melodic]([http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)) 

### Compile and execute package
```
rosmake wam_robot
```

```
roslaunch wam_node wam_node.launch
```
	
### Example of running the services
**Move BHand Fingers:**
```
rosservice call /bhand/finger_pos "radians:
 0.0
 0.0
 0.0" 
```

**Move WAM Joints:**
```
rosservice call /wam/joint_move "joints:
 0.0
 0.0
 0.0
 0.0
 0.0
 0.0
 0.0"
```
**Move WAM to Tool Pose:**
```
rosservice call /wam/pose_move "pose:
  position:
    x: 0.048
    y: 0.0
    z: 0.618
  orientation:
    x: -0.0190
    y: 0.9022
    z: -0.2516
    w: -0.3498"
```
**Move WAM Home:**
```
rosservice call /wam/go_home
```
**Hold Joint Positions:**
```
rosservice call /wam/hold_joint_pos "hold: true"
```

**Unhold Joint Positions:**
```
rosservice call /wam/hold_joint_pos "hold: false"
```
