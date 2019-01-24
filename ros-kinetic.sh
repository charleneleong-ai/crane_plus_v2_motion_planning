#!/bin/bash -i

printf "\n==========Updating Ubuntu==========\n"
sudo apt-get update -y
sudo apt-get upgrade -y

#  http://wiki.ros.org/kinetic/Installation/Ubuntu
printf "\n==========Installing ROS Kinetic==========\n"
# Setup sources list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keyserver
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
# Install full ros kinetic
sudo apt-get update && sudo apt-get install ros-kinetic-desktop-full -y
# Installing dependencies for building packages
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential ros-kinetic-gazebo9-* -y

# http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
printf "\n==========Configuring ROS Kinetic==========\n"
sudo rosdep init 
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

printf "\n==========Creating ROS Workspace==========\n"
# Creating and building catkin workspace
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ 
catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc

printf "\n==========ROS Environment Variables==========\n"
# Confirming installation
printenv | grep ROS

