#!/usr/bin/env python

import sys
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

from parameter_tuning_session import ParamTuningSession


def main():

    rospy.init_node('parameter_tuning', anonymous=True)
    # poses = rospy.get_param("/parameter_tuning/poses")
    session = ParamTuningSession()
    session.run()
    # session.get_results()


if __name__ == "__main__":
    main()
