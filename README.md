# barrett-ros-pkg
> ROS package for the Barrett WAM and related products.
- [Overview](#overview)
 - [ Pre-Requisites](#pre-requisites)
 - [Compiling the package](#compiling-the-package)
 - [Running `wam_node`](#running-wam_node)
 - [Running `perception_palm`](#running-perception_palm)
	 - [Set up cameras](#set-up-cameras)
	 - [Calibration](#calibration)
	 - [Running the demo](#running-the-demo)
	 - [Accessing the sensors](#accessing-the-sensors)
		 - [Viewing with rviz](#viewing-with-rviz)
		 - [LED](#led)
		 - [Laser](#laser)
		 - [IR Range finder](#ir-range-finder)
		 - [Camera](#camera)
	 - [Networking](#networking)
	 - [Troubleshooting](#troubleshooting)
 - [Running `barrett_hand_node`](#running-barrett_hand_node)
 - [Example of running the services (Tested on ROS Melodic and Indigo)](#example-of-running-the-services-tested-on-ros-melodic-and-indigo)

## Overview
This is Barrett Technology's ROS repository that wraps Libbarrett's functionalities and includes a ROS driver for Barrett's Perception Palm. The Perception Palm includes a LED, a Laser, two cameras and an IR Range finder. The driver for the Perception palm wraps the open source C/C++ library for Microchip's USB-to-SPI protocol coverter. Libbarrett is a real-time controls library written in C++ that runs Barrett Technology's products, including the WAM Arm and the BH8-282 BarrettHand. 

- The `wam_node` stack is designed to be run on a WAM PC-104 or external control PC, and can work with the WAM with any combination of the wrist, BarrettHand and the Force/Torque sensor attached.
- The `barrett_hand_node` stack is designed to run on an external control PC and can work with the BarrettHand standalone.
- The `perception_palm` stack is designed to run on an external control PC.
- The `wam_msgs` and `wam_srvs` stacks are designed as the interface to communicate with the functionality exposed by the` wam_node`.

## Pre-Requisites
#### On  Ubuntu 14.04:
1.  An installed version of [Libbarrett 1.3.0](https://git.barrett.com/software/libbarrett/blob/release/release-1.3.0/README.txt)
2. [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) 
3. Install libudev and wstool.
```sh
sudo apt-get update
sudo apt-get install libudev-dev
sudo apt-get install python-wstool
```
4. Install the camera driver:
```
sudo apt-get install ros-indigo-camera-umd
```
#### On  Ubuntu 18.04:
1. An installed version of [Libbarrett 2.0.0](https://git.barrett.com/software/libbarrett/blob/devel/README.md)
2. [ROS Melodic]([http://wiki.ros.org/melodic/Installation/Ubuntu](http://wiki.ros.org/melodic/Installation/Ubuntu)) 
3. Install libudev and wstool.
```sh
sudo apt-get update
sudo apt-get install libudev-dev
sudo apt-get install python-wstool
```
4. Install the camera driver:
```
sudo apt-get install ros-melodic-camera-umd
```
## Compiling the package
Create a new Catkin Workspace **if not already done**:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
Clone the git repository an run the build script:
```
cd ~/catkin_ws/src
git clone https://git.barrett.com/software/barrett-ros-pkg.git
./build.sh
```

Source the package before running, or add it to ```bashrc```:
```sh
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```
## Running `wam_node`
Launch the ```wam_node.launch``` file, **with the WAM Arm connected**:
```sh
roslaunch wam_node wam_node.launch
```
## Running `perception_palm`
### Set up cameras:
1. **Connect the Perception Palm to the PC** before completing the following steps.

2. Edit the launch file to confirm camera setup parameters.
```
gedit ~/catkin_ws/src/barrett-ros-pkg/perception_palm/launch/perception_palm.launch
```
    1. By default, only one camera is used. To use both cameras, uncomment the stereo camera node and comment the monocular camera node to use both the cameras. Example:
    ```
    <!--
    commented lines
    -->
    ```

    2. Ensure that the launch file targets the correct devices. By default, the two cameras are `/dev/video0` and `/dev/video1`. However, if you have other cameras on your system, this may be different. List the video devices with
    ```
    ls /dev/video*
    ```
    to see if you have extra video devices. To determine which devices are correct, you can use a program such as `cheese`:
    ```
    sudo apt-get install cheese
    cheese
    ```
    Under Edit > Preferences > Device, you can test each device to determine which is which. If you need to change the default device(s), edit the lines in the launch file that look like this:
    ```
    <param name="device" type="string" value="/dev/video0" />
    ```
    
    3. Save and exit the editor.

4. Load the correct camera module. For one camera
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo
```
    or two cameras
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo quirks=128
```

*Notes*

Two cameras can be used simultaneously at a maximum resolution of 320 x 240 and a single camera can be used at a maximum resolution of 1600 x 1200. For information on maximum camera resolutions, refer to the spec sheet.

The camera with the red filter is physically installed with 180 degrees shift. So, in this configuration the camera with red filter is rotated by 180 degrees. Make sure that the appropriate camera (left/right) is shifted while configuring based on the corresponding device ennumerations (/dev/video0 or /dev/video1). The necessary changes can be made in the launch/perception_palm.launch file.

### Calibration

**IR Range finder**

**The IR range finder needs to be calibrated for the first time before using it.** It will work without calibration but it might not be accurate. Follow the instructions below to calibrate it. This step can be skipped if you do not want to use the IR or if the IR range finder is already calibrated.
```
cd ~/catkin_ws/src/barrett-ros-pkg/perception_palm/include/MCP2210-Library
make
sudo ./dist/Debug/GNU-Linux-x86/ir_calibrate
```
Follow the onscreen instructions.

**Cameras**

Please refer to the ROS [Stereo](http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration)/[Monocular](http://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration) calibration package. This step is completely optional. The cameras would work even if this step is skipped.

### Running the demo
1. Configure the cameras (every time you plug in the Perception Palm or reboot the computer). For one camera
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo
```
    or two cameras
```
sudo rmmod uvcvideo
sudo modprobe uvcvideo quirks=128
```

2. Become the root user to access the drivers and run the demo.
```
sudo -s
source ~/catkin_ws/devel/setup.bash
roslaunch perception_palm perception_palm.launch
```

3. To quit, press Ctrl-C. Then type `exit` to return to a regular terminal.

*Troubleshooting*

If the camera node fails to start in step 2, make sure the configuration you chose in step 1 matches the configuration in the launch file. See the "Set up cameras" section for details.

### Accessing the sensors
While the demo is running you can access the sensors from a separate terminal.

#### Viewing with rviz

ROS has a visualization tool called `rviz` that allows you to view sensor data and video. It can be run for mono or stereo camera configurations by
```
rviz -d ~/catkin_ws/src/barrett-ros-pkg/perception_palm/launch/palm_mono.rviz
rviz -d ~/catkin_ws/src/barrett-ros-pkg/perception_palm/launch/palm_stereo.rviz
```

*Note: If you encounter a segmentation fault when launching rviz, make sure you're using the correct configuration file, then wait a moment and try again.*

#### LED

The LED can be turned off and on by calling the service barrett/palm/set_led_on.<br />
```	
rosservice call barrett/palm/set_led_on ['True']
rosservice call barrett/palm/set_led_on ['False']
```

#### Laser

The Laser can be turned off and on by calling the service barrett/palm/set_laser_on.<br />
```	
rosservice call barrett/palm/set_laser_on ['True']
rosservice call barrett/palm/set_laser_on ['False']
```

#### IR Range finder

The IR range finder publishes the range information as a sensor_msgs/Range message, at 1Hz to the topic barrett/palm/ir/range<br />
```
rostopic echo barrett/palm/ir/range
```

#### Camera

By default, the two camera feeds are published at the rate of 30 fps with a resolution of 320x240 px. They are published to the topics, barrett/palm/left/image_raw and barrett/palm/right/image_Raw.<br />
While using the monocular camera mode, the images are published at the rate of 30 fps with a resolution of 1600 x 1200 to the topic barrett/palm/image_raw.<br />

### Networking
This is required if you use run the ROS nodes across multiple computers. The below instructions are to setup the perception palm on the XWAM and to control it from a remote host.

In the host computer, open a new terminal and ssh into the XWAM
```
ssh summit@*xwam's ip address*
```
Enter the XWAM's password and open the hosts file
```
sudo vi /etc/hosts
```
Add the IP address of the remote host machine to the list and name it
```
*remote host's ip address* *remote host's name*
```
Save and exit it. Set the XWAM to be the ROS MASTER
```
export ROS_MASTER_URI=http://*XWAM's ip address*:11311
```
Open a new terminal and edit the host file in the host machine as above
```
sudo vi /etc/hosts
```
In the host machine, name the IP address of the xwam
```
*xwam's ip address* summit
```
Save and exit it. Update the ROS_MASTER as the XWAM in the host machine
```
export ROS_MASTER_URI=http://*XWAM's ip address*:11311
```

P.S: The ROS_MASTER_URI variable would be active as long as the terminal is open. If the terminal is closed and restarted, the ROS_MASTER_URI needs to be updated.
For more information on ROS network configuration refer to [ROS documentation](http://wiki.ros.org/ROS/NetworkSetup).

### Troubleshooting
Trying to restart the perception_palm package multiple times might fail with an error "Error setting SPI Parameters".
This can be solved by unplugging and plugging back the USB to the port.

## Running `barrett_hand_node`
Launch the ```barrett_hand_node.launch``` file, with the **BarrettHand connected**:
```sh
roslaunch barrett_hand_node barrett_hand_node.launch
```
## Example of running the services (Tested on ROS Melodic and Indigo)
**Move BHand Fingers:**
```
rosservice call /bhand/finger_pos "radians:
- 0.0
- 0.0
- 0.0" 
```

**Move WAM Joints:**
```
rosservice call /wam/joint_move "joints:
- 0.0
- 0.0
- 0.0
- 0.0"
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
