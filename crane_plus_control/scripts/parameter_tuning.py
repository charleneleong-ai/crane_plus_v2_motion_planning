#!/usr/bin/env python

import sys
<<<<<<< HEAD
import rospy

import moveit_commander
=======
from timeit import default_timer as timer
from collections import OrderedDict

import csv
import pprint
import pandas as pd
import numpy as np

import rospkg
import rospy

import moveit_commander
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
import moveit_msgs.msg

from hyperopt import hp, rand, tpe, Trials, fmin, STATUS_OK
from hyperopt.pyll.stochastic import sample

>>>>>>> dev
from parameter_tuning_session import ParamTuningSession


def main():
<<<<<<< HEAD
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--max_trials', type=int, default=30,
    #                     help='Max num of trials for hyperopt')
    # args = parser.parse_args()
    moveit_commander.roscpp_initialize(sys.argv)
=======

>>>>>>> dev
    rospy.init_node('parameter_tuning', anonymous=True)
    # poses = rospy.get_param("/parameter_tuning/poses")
    session = ParamTuningSession()
    session.run()
    # session.get_results()

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
