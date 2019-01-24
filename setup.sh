#!/bin/bash -i

printf "\n==========  Updating Ubuntu  ==========\n"
sudo apt-get update && sudo apt-get upgrade -y


printf "\n==========  Installing ROS Kinetic  ==========\n"
#  http://wiki.ros.org/kinetic/Installation/Ubuntu
# Setup sources list
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
# Setup keyserver
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
# Install full ros kinetic
sudo apt-get update && sudo apt-get install ros-kinetic-desktop-full -y
# Installing dependencies for building packages
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

# Gazebo 9  
# http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
sudo apt-get update
sudo apt-get install gazebo9 ros-kinetic-gazebo9-* -y


# Configuring ROS Kinetic
# http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
sudo rosdep init &&  rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

printf "\n==========  Creating ROS Workspace  ==========\n"
# Creating and building catkin workspace
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc

# Building repo and dependencies
cd ~/catkin_ws/src/ 
git clone http://gojou/gitlab/charyeezy/crane_plus_v2_motion_planning.git || { echo message && exit 1; }
cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 
cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash

printf "\n==========  Installing Parameter Tuning Dependencies  ==========\n"
# pip
sudo apt-get install python-pip python3-pip
source ~/.bashrc

# hyperopt, bayesopt
cd ~/catkin_ws/src/crane_plus_v2_motion_planning && pip install -r requirements.txt

# SMAC3
# https://automl.github.io/SMAC3/master/installation.html
sudo apt-get install build-essential swig
curl https://raw.githubusercontent.com/automl/smac3/master/requirements.txt | xargs -n 1 -L 1 pip3 install
pip3 install smac
find ./ -name "smac3_run.py" -exec chmod u+x {} \;

# find ./ -name "*.py" -exec chmod +x {} \;
# Clean
sudo apt-get autoremove -y

# Confirming installation
printf "\n==========  ROS Environment Variables  ==========\n"
printenv | grep ROS


