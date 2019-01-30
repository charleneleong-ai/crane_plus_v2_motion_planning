#!/bin/bash -i

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

CYAN "\n==========  Warehouse ROS MongoDB and dependencies ==========\n"
# https://github.com/ros-planning/warehouse_ros_mongo
blue "Cloning warehouse_ros_mongo and mongo-cxx-driver git repos"
cd ~/catkin_ws/src && git clone https://github.com/ros-planning/warehouse_ros_mongo.git { echo message && exit 1; }
git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git
blue "Installing scons"
sudo apt-get install scons -y
blue "Compiling mongo-cxx-driver with scons"
cd mongo-cxx-driver && sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors

BLUE "Recompiling catkin ws"
cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash 

BLUE "Autoremove unnecessary packages"
sudo apt-get autoremove -y

CYAN "\n==========  BENCHMARK DB INSTALLATION COMPLETE!!!  ==========\n"
