

## Crane+V2 Robot Arm

Code and models for the [Crane+V2 5DOF](https://www.rt-net.jp/products/cranep2?lang=en) (4DOF arm + gripper) with the [MoveIt! Framework](http://moveit.ros.org/) in ROS.

Based on the [TurtleBot Arm code](https://github.com/turtlebot/turtlebot_arm).



#### Robot Arm Description

<div style="width:image width px; font-size:80%; text-align:center;">
<img src="imgs/cranev2_tf.png" width="600" align="middle"/></div>


[Source](https://www.rt-shop.jp/blog/archives/6711)

#### Motion Planning with MoveIt! Framework in RViz 

<div style="width:image width px; font-size:80%; text-align:center;">
<img src="imgs/cranev2_rviz.png" width="600" align="middle"/></div>


## Requirements

- [Ubuntu 16.04 Xenial](http://releases.ubuntu.com/16.04/)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- Python 2.7

This package has only been tested on Ubuntu 16.04 with ROS Kinetic which is only stable on Python 2.7 due to compatibility with packages. 

If you wish to run ROS along with Python 3 in Anaconda, please [create a py2.7 conda environment](https://www.youtube.com/watch?v=EMF20z-gT5s) for ROS.

## Packages

**camera_plus_control**

​	Parameter tuning and benchmarking nodes

**camera_plus_description**

​	CAD files and [URDF](http://wiki.ros.org/urdf) (Unified Robot Description Format) model of CRANE+V2

**camera_plus_gripper**

​	Node that controls the gripper of CRANE+V2

**camera_plus_hardware**

​	Launch file that launches and configures the settings for use with CRANE+V2 hardware

**camera_plus_ikfast_arm_plugin**

​	Inverse kinematics plugin for CRANE+V2 in the MoveIt! framework

**camera_plus_joint_state_publisher**

​	Node that converts servo status messages (`dynamixel_msgs/JointState` type) output by the Dynamixel servo controller to ROS `sensor_msgs/JointState` type

**camera_plus_moveit_config**

​	Parameters and launch files for using CRANE+V2 with MoveIt! framework

**camera_plus_simulation**

​	Launch file that launches and configures the settings for simulating CRANE+V2 in Gazebo



## Contents

1. [Quick Launch](#quick-launch)
2. [ROS Installation and Configuration](#ros-installation-and-configuration)
3. [Parameter Tuning](./crane_plus_control/README.md)

## Quick Launch

Please see [instructions below](#ros-installation-and-configuration) for first time ROS installation and configuration.


1. Download the project files, install all dependent packages and compile the project.

    ```bash
    $ cd ~/catkin_ws/src/ && git clone http://gojou/gitlab/charyeezy/crane_plus_v2_motion_planning.git
    $ cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 
    $ catkin_make && source ~/catkin_ws/devel/setup.bash
    ```

2. Launch the CRANE+V2 robot model either through the hardware interface or through simulation.

    Launch robot model through the hardware interface.

    ```bash
    $ roslaunch crane_plus_hardware start_arm_standalone.launch
    ```

    Launch robot model through simulation.

    ```bash
    $ roslaunch crane_plus_simulation simulation.launch
    ```

3. Control through MoveIt RViz.  Set `robot_execution:=true` to launch on real robot or Gazebo simulation.

    ```bash
    $ roslaunch crane_plus_moveit_config crane_plus.launch robot_execution:=true
    ```

    [OPTIONAL] Control headlessly for named poses. You can optionally set `rviz:=false` .

    Select from list of named states: [vertical, backbend, resting, low_fwd_reach, pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9, pose10, pose11, pose12, pose13, pose14]

    ```bash
    $ roslaunch crane_plus_moveit_config crane_plus.launch robot_execution:=true <rviz:=false>
    $ roslaunch crane_plus_control named_pose pose:=<pose>
    ```



## ROS Installation and Configuration

1. [Install](http://wiki.ros.org/kinetic/Installation/Ubuntu) ROS Kinetic 

    ```bash
    $ sudo apt-get update && sudo apt-get upgrade -y
    $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
    $ sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
    $ sudo apt-get update && sudo apt-get install ros-kinetic-desktop-full -y
    $ sudo rosdep init && rosdep update
    $ echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc && source ~/.bashrc
    $ sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y
    ```

2. [Configure](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment) your ROS environment

    ```bash
    $ mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/ && catkin_make
    $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc
    ```

3. Confirm the installation and configuration of ROS. You should see the following output.

    ```bash
    $ printenv | grep ROS
    ROS_ROOT=/opt/ros/kinetic/share/ros
    ROS_PACKAGE_PATH=/home/<username>/catkin_ws/src:/opt/ros/kinetic/share
    ROS_MASTER_URI=http://localhost:11311
    ROS_VERSION=1
    ROSLISP_PACKAGE_DIRECTORIES=/home/<username>/catkin_ws/devel/share/common-lisp
    ROS_DISTRO=kinetic
    ROS_ETC_DIR=/opt/ros/kinetic/etc/ros
    ```

