

## Crane+V2 Robot Arm

Code and models for the [Crane+V2](https://www.rt-net.jp/products/cranep2?lang=en) (4DOF arm + gripper) with the [MoveIt! Motion Planning Framework](http://moveit.ros.org/) for [ROS](http://wiki.ros.org/).

![](imgs/crane_plus_moveit.png)



## Contents

- [Environment Setup](#environment-setup)
- [Package Description](#package-description)
- [Quick Start](#quick-start)



## Environment Setup

- [Ubuntu 16.04 Xenial](http://releases.ubuntu.com/16.04/)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
- [Python 2.7](https://www.python.org/download/releases/2.7/)

This package has only been tested on Ubuntu 16.04 with ROS Kinetic. ROS is currently only stable on Python 2.7 due to compatibility with legacy packages. 

If you do not have Ubuntu16.04, an option is to download the Ubuntu16.04 [ISO](http://releases.ubuntu.com/16.04/ubuntu-16.04.5-desktop-amd64.iso) and install on a VM such as [VMWare Workstation Player](https://www.youtube.com/watch?v=Wmx5hZ_m7EY) or [Virtualbox](https://www.youtube.com/watch?v=RBU1xMP-SGc).

If you wish to run ROS along with Python 3 in Anaconda, please [create a py2.7 conda environment](https://www.youtube.com/watch?v=EMF20z-gT5s) for ROS.

```bash
$ conda create -n ros_env python=2.7 anaconda -y
$ source activate ros_env
(ros_env) $ pip install -U catkin_pkg rospkg
```



## Package Description

**crane_plus_control:** Parameter tuning and benchmarking nodes

**crane_plus_description:** [CAD files](./crane_plus_description/README.md) and [URDF](http://wiki.ros.org/urdf) (Unified Robot Description Format) model of CRANE+V2

**crane_plus_gripper:** Node that controls the gripper of CRANE+V2

**crane_plus_hardware:** Launch file that configures the settings for use with CRANE+V2 hardware

**crane_plus_ikfast_arm_plugin:** Custom inverse kinematics plugin for CRANE+V2 in the MoveIt! framework

**crane_plus_joint_state_publisher:** Node that converts servo status messages ([`dynamixel_msgs/JointState`](http://docs.ros.org/kinetic/api/dynamixel_msgs/html/msg/JointState.html) message type) output by the Dynamixel servo controller to ROS [`sensor_msgs/JointState`](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/JointState.html) message type

**crane_plus_moveit_config:**Parameters and launch files for using CRANE+V2 with MoveIt! framework

**crane_plus_simulation:** Launch file that configures the settings for simulating CRANE+V2 in Gazebo



## Quick Start


1. Download and run the [`ros-kinetic.sh`](./ros-kinetic.sh) script for first time ROS installation and configuration.

    ```bash
    $ curl --noproxy "*" -L -O 'http://gojou/gitlab/charyeezy/crane_plus_v2_motion_planning/raw/master/ros-kinetic.sh'
    $ chmod u+x ros-kinetic.sh && ./ros-kinetic.sh 
    ```

2. Download the project files, install all dependent packages and compile the project. If you are working with a conda env, remember to `source activate ros_env` before installing pip requirements and before using `roslaunch`. 

    ```bash
    $ cd ~/catkin_ws/src/ && git clone http://gojou/gitlab/charyeezy/crane_plus_v2_motion_planning.git 
    $ cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 
    $ cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash
    ```

3. Launch the CRANE+V2 robot model either through the hardware interface or through simulation.

    Launch robot model through the hardware interface.

    ```bash
    $ roslaunch crane_plus_hardware start_arm_standalone.launch
    ```

    Launch robot model through simulation.

    ```bash
    $ roslaunch crane_plus_simulation simulation.launch
    ```

4. Control through MoveIt RViz.  Set `robot_execution:=true` to launch on real robot or Gazebo simulation.

    ```bash
    $ roslaunch crane_plus_moveit_config crane_plus.launch robot_execution:=true
    ```

    [OPTIONAL] Control headlessly for named poses. You can optionally set `rviz:=false` .

    Select from list of named states: [vertical, backbend, resting, low_fwd_reach, pose1, pose2, pose3, pose4, pose5, pose6, pose7, pose8, pose9, pose10, pose11, pose12, pose13, pose14]

    ```bash
    $ roslaunch crane_plus_moveit_config crane_plus.launch robot_execution:=true rviz:=false
    $ roslaunch crane_plus_control named_pose.launch pose:=resting
    ```