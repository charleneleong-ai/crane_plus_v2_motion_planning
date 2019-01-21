#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Last Modified: Monday, January 21st 2019, 4:10:10 pm
# Modified By: Charlene Leong
###

# from session import Session
# from __future__ import scipy
import sys
import os

import rospkg

from session import Session

ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/scripts/modules'

# class SMACSession(Session):
#     def __init__(self, mode):
#         super(SMACSession, self).__init__(mode)


# def run(self):
    
#     # os.system("smac/smac --scenario-file " + name)
#     os.system("smac/smac --scenario-file example_scenarios/branin/branin-scenario.txt --seed 1234")
name = ROS_PKG_PATH+"/scenarios/tmpscenario.txt"

planner_type = "geometric::BKPIECE"
pcs_file = "BKPIECE_better.pcs"
scene_number = "narrow_passage"
problem_iterations = "1"
smac_iterations = 10000
walltime_limit = 30*60

if __name__ == '__main__':
    try:
        tfile = open(name,'a') 
        tfile.write("use-instances = false\n")
        tfile.write("numberOfRunsLimit = " + str(smac_iterations) + "\n")
        tfile.write("runtimeLimit = " + str(walltime_limit) + "\n")
        tfile.write("runObj = QUALITY\n")
        tfile.write("deterministic = 1\n")
        tfile.write("pcs-file = pcs/" + pcs_file + "\n")
        tfile.write("algo = python smac_runproblem.py " + planner_type + " " + scene + " " + problem_iterations + "\n")
        
        tfile.write("check-sat-consistency false \n")
        tfile.write("check-sat-consistency-exception false \n")
        tfile.write("algo-cutoff-time 40\n")
        tfile.write("kill-run-exceeding-captime-factor 2\n")

        # tfile.write("kill-run-exceeding-captime false")
        # tfile.write("transform-crashed-quality false")

        tfile.close()
    except:
        print('Something went wrong!')
        sys.exit(0)
        
    # os.system("smac/./smac --scenario-file example_scenarios/branin/branin-scenario.txt --seed 1234")
