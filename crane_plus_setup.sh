#!/bin/bash -i
# Needs to be interactive shell for source ~/.bashrc

BLUE=`tput bold && tput setaf 4`
CYAN=`tput bold && tput setaf 6`
blue=`tput setaf 4`
NC=`tput sgr0`

function BLUE(){
	echo -e "\n${BLUE}${1}${NC}"
}
function CYAN(){
	echo -e "\n${CYAN}${1}${NC}"
}
function blue(){
	echo -e "\n${blue}${1}${NC}"
}


CYAN " \n==========  Updating Ubuntu  ==========\n"
blue "sudo apt-get update"
sudo apt-get update -y
blue "sudo apt-get upgrade"
sudo apt-get upgrade -y


CYAN "\n==========  Installing ROS Kinetic  ==========\n"
#  http://wiki.ros.org/kinetic/Installation/Ubuntu
blue "Setup sources list"
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
blue "Setup keyserver"
echo "" | sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
blue "sudo apt-get update"
sudo apt-get update -qq
blue "Installing ros-kinetic-desktop"
sudo apt-get install ros-kinetic-desktop -y || { echo message && exit 1; }

BLUE "Installing dependencies for building packages"
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y || { echo message && exit 1; }

BLUE "Configuring ROS Kinetic"
# http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
blue "Initialising rosdep"
sudo rm -r /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init &&  rosdep update || { echo message && exit 1; }
blue "Adding to ~/.bashrc"
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc || { echo message && exit 1; }
source ~/.bashrc 

blue "Creating and building catkin workspace"
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ && catkin_make || { echo message && exit 1; }

blue "Adding to ~/.bashrc"
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
source ~/.bashrc

CYAN "\n==========  Installing Gazebo 9 ==========\n"
BLUE "Installing Gazebo 9"
# http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0
blue "Setup sources list"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
blue "Setup keys"
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
blue "sudo apt-get update"
sudo apt-get update -qq

BLUE "Installing ROS Kinetic Gazebo 9"
sudo apt-get install gazebo9 ros-kinetic-gazebo9-* -y || { echo message && exit 1; }


CYAN "\n==========  Installing CRANE V2+ and dependencies ==========\n"
BLUE "Cloning crane_plus_v2_motion_planning Git repo and submodules"
cd ~/catkin_ws/src/ && git clone https://github.com/charyeezy/crane_plus_v2_motion_planning.git || { echo message && exit 1; }

BLUE "Installing ROS dependencies"
cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic || { echo message && exit 1; }

BLUE "Recompiling catkin ws"
cd ~/catkin_ws && catkin_make && source ~/.bashrc || { echo message && exit 1; }


BLUE "Autoremove unnecessary packages"
sudo apt-get autoremove -y

# Confirming installation
CYAN "\n==========  ROS Environment Variables  ==========\n"
printenv | grep ROS


CYAN "\n==========  CRANE V2+ INSTALLATION COMPLETE!!!  ==========\n"



