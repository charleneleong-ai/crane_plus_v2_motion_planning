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


CYAN "\n==========  Installing CRANE V2+ Parameter Tuning Dependencies  ==========\n"
BLUE "Installing latest pip"
sudo apt-get install python-pip python3-pip -y
# pip install --user --upgrade pip
source ~/.bashrc

#export http_proxy="melinet:9515"
#wget https://bootstrap.pypa.io/get-pip.py
#sudo python get-pip.py
#sudo python3 get-pip.py

BLUE "Installing pip requirements"
cd ~/catkin_ws/src/crane_plus_v2_motion_planning/crane_plus_control && pip install --user -r requirements.txt

BLUE "Installing submodules"
cd ~/catkin_ws/src/crane_plus_v2_motion_planning && git submodule update --init --recursive

BLUE "Installing SMAC3 requirements"
# https://automl.github.io/SMAC3/master/installation.html
cd ~/catkin_ws/src/crane_plus_v2_motion_planning/crane_plus_control/scripts/modules/SMAC3
sudo apt-get install swig -y
pip3 install --user pybind11
cat requirements.txt | xargs -n 1 -L 1 pip3 install --user
blue "python3 setup.py install"
sudo python3 setup.py install
blue "Fixing error in smac"
cd scripts && cat smac | sed 's/python/python3/' 


BLUE "Installing OpenTuner requirements"
cd ~/catkin_ws/src/crane_plus_v2_motion_planning/crane_plus_control/scripts/modules/opentuner
sudo apt-get install `cat debian-packages-deps | tr '\n' ' '` -y
pip install --user opentuner


BLUE "Autoremove unnecessary packages"
sudo apt-get autoremove -y

CYAN "\n==========  PARAMETER TUNING INSTALLATION COMPLETE!!!  ==========\n"



