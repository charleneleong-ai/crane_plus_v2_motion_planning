#!/usr/bin/env python
###
# File Created: Monday, January 21st 2019, 10:55:57 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Last Modified: Friday, January 25th 2019, 9:35:09 am
# Modified By: Charlene Leong
###


import sys
import os
import time
import logging
import pprint

import signal

import numpy
#import cPickle as pickle

import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs
import shape_msgs
import std_msgs
from datetime import datetime as dt
import sys
import math
import random

from session import Session

logging.basicConfig(level=logging.INFO)


class SMACRun(Session):
    def __init__(self):
        super(SMACRun, self).__init__()


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
    params = sys.argv[9:]
    configMap = dict((name[1:], value) for name, value in zip(params[::2], params[1::2]))
    pprint.pprint(configMap)

    quality = 1000.0

    signal.signal(signal.SIGINT, sigint_exit)

    quality = bmclass.bm_run_cost(problem_config, configMap)

    print ("Result of algorithm run: SUCCESS, 0, 0, %f, 0" % 1)
