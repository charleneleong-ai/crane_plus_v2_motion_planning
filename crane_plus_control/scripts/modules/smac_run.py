#!/usr/bin/env python
###
# File Created: Monday, January 21st 2019, 10:55:57 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Last Modified: Friday, January 25th 2019, 11:55:48 am
# Modified By: Charlene Leong
###


import sys
import os
import time
import logging
import pprint
import signal

import numpy

import rospkg
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs
import shape_msgs
import std_msgs

import sys
import math
import random

from session import Session

logging.basicConfig(level=logging.INFO)

ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control') 

class SMACRun(Session):
    def __init__(self, scene):
        self._set_rosparams()

        super(SMACRun, self).__init__()
        
        self.scenes[scene]

    # Porting params from SMACSession
    def _set_rosparams(self):
        session_params = rospy.get_params('/parameter_tuning/'))
        

    def _objective(self, planner, params):
        self.n_trial += 1

        # Set new params
        self.planner_config_obj.set_planner_params(planner, params)

        if self.path_tune:
            results = super(SMACRun, self)._get_stats(
                self.start_pose, self.target_pose)
            stats = {'t_avg_run_time': results['avg_run_time'], 't_avg_plan_time': results['avg_plan_time'],
                     't_avg_dist': results['avg_dist'], 't_avg_path_length': results['avg_path_length'], 't_avg_success': results['avg_success']}
        else:
            results, stats = super(SMACRun, self).run_problem_set(planner_id=planner)




        
def sigint_exit(signal, frame):
	moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('smac_run', anonymous=True, log_level=rospy.FATAL)

    # Read in first arguments.
    planner = sys.argv[1]
    scene = sys.argv[2]
    max_trials = sys.argv[3]

    instance = sys.argv[4]
    specifics = sys.argv[5]
    cutoff = int(float(sys.argv[6]) + 1)
    runlength = int(sys.argv[7])
    seed = int(sys.argv[8])


    # Read in parameter setting and build a dictionary mapping param_name to param_value.
    params_args = sys.argv[9:]
    params = dict((name[1:], value) for name, value in zip(params_args[::2], params_args[1::2]))
    pprint.pprint(configMap)

    smac_run = SMACRun(scene)

    result_log, stats = smac_run.run_problem_set(planner, )

    quality = 1000.0

    signal.signal(signal.SIGINT, sigint_exit)

    quality = bmclass.bm_run_cost(problem_config, configMap)

    print ("Result of algorithm run: SUCCESS, 0, 0, %f, 0" % 1)
