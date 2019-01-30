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

chmod u+x crane_plus_setup.sh && ./crane_plus_setup.sh 

cd ~/catkin_ws/src/crane_plus_v2_motion_planning/crane_plus_control
chmod u+x parameter_tuning_setup.sh && ./parameter_tuning_setup.sh 

cd ~/catkin_ws/src/crane_plus_v2_motion_planning/crane_plus_control
chmod u+x benchmark_db_setup.sh && ./benchmark_db_setup.sh 


CYAN "\n==========  CRANE V2+ FULL INSTALLATION COMPLETE!!!  ==========\n"



