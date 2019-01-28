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
blue "Installing ros-kinetic-desktop"
sudo apt-get install ros-kinetic-desktop -y

BLUE "Installing dependencies for building packages"
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential -y

BLUE "Installing Gazebo 9"
# http://gazebosim.org/tutorials?cat=install&tut=install_ubuntu&ver=9.0
blue "Setup sources list"
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
blue "Setup keys"
wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - 
blue "sudo apt-get update"
sudo apt-get update -qq

BLUE "Installing ROS Kinetic Gazebo 9"
sudo apt-get install gazebo9 ros-kinetic-gazebo9-* -y

BLUE "Configuring ROS Kinetic"
# http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
blue "Initialising rosdep"
sudo rosdep init &&  rosdep update
blue "Adding to ~/.bashrc"
if [ $? -eq 1 ]
then
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
	source ~/.bashrc
fi

blue "Creating and building catkin workspace"
mkdir -p ~/catkin_ws/src 
cd ~/catkin_ws/ && catkin_make

blue "Adding to ~/.bashrc"
if [ $? -eq 1 ]
then
	echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc 
	source ~/.bashrc
fi

BLUE "Cloning crane_plus_v2_motion_planning Git repo"
cd ~/catkin_ws/src/ && git clone http://gojou/gitlab/charyeezy/crane_plus_v2_motion_planning.git || { echo message && exit 1; }
#git clone git@github.com:charyeezy/crane_plus_v2_motion_planning.git || { echo message && exit 1; }
cd ~/catkin_ws/src/crane_plus_v2_motion_planning && git checkout smac3

BLUE "Installing ROS dependencies"
cd ~/catkin_ws && rosdep install -y --from-paths src --ignore-src --rosdistro kinetic 

BLUE "Installing Warehouse ROS Mongo DB dependencies"
# https://github.com/ros-planning/warehouse_ros_mongo
blue "Cloning warehouse_ros_mongo and mongo-cxx-driver git repos"
cd ~/catkin_ws/src && git clone https://github.com/ros-planning/warehouse_ros_mongo.git
git clone -b 26compat https://github.com/mongodb/mongo-cxx-driver.git
blue "Installing scons"
sudo apt-get install scons -y
blue "Compiling mongo-cxx-driver with scons"
cd mongo-cxx-driver && sudo scons --prefix=/usr/local/ --full --use-system-boost --disable-warnings-as-errors

cd ~/catkin_ws && catkin_make && source ~/catkin_ws/devel/setup.bash

CYAN "\n==========  Installing Parameter Tuning Dependencies  ==========\n"
BLUE "Installing pip"
sudo apt-get install python-pip python3-pip -y
source ~/.bashrc

BLUE "Installing pip requirements"
cd ~/catkin_ws/src/crane_plus_v2_motion_planning && pip install -r requirements.txt

BLUE "Installing SMAC3"
# https://automl.github.io/SMAC3/master/installation.html
blue "Cloning SMAC3 Git repo"
cd ~/catkin_ws/src/crane_plus_v2_motion_planning/crane_plus_control/scripts/modules
git clone https://github.com/automl/SMAC3.git && cd SMAC3
blue "Installing requirements"
cat requirements.txt | xargs -n 1 -L 1 pip3 install 
blue "python3 setup.py install"
sudo python3 setup.py install
cd ~/catkin_ws/src/crane_plus_v2_motion_planning/crane_plus_control/scripts/modules/SMAC3/scripts
blue "Fixing error in smac"
cat smac | sed 's/python/python3/' 	

#cd ~/catkin_ws/src && sudo find ./ -name "*.py" -exec chmod u+x {} \;
#sudo find ./ -name "smac" -exec chmod u+x {} \;

BLUE "Autoremove unnecessary packages"
sudo apt-get autoremove -y

# Confirming installation
CYAN "\n==========  ROS Environment Variables  ==========\n"
printenv | grep ROS

CYAN "\n==========  CRANE V2+ INSTALLATION COMPLETE!!!  ==========\n"



