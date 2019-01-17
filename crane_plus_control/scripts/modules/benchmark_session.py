#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 2:10:34 pm
# Modified By: Charlene Leong
# Last Modified: Thursday, January 17th 2019, 3:17:25 pm
# Author: Charlene Leong (charleneleong84@gmail.com)
###


import numpy
import cPickle as pickle
import pprint
import json

import rospkg
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs
import shape_msgs
import std_msgs
from datetime import datetime as dt

from session import Session


ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/scripts'


class BenchmarkSession(Session):
    """Constructor for benchmarking session

    Args:
        Session (object): Session object initiates session with default functions
    """

    PLANNING_TIME = 2  # seconds

    def __init__(self, mode):
        super(BenchmarkSession, self).__init__(mode)
        rospy.loginfo('Initialising benchmarking session in %s mode', mode)


    def run(self):
        self.group.set_planning_time(self.PLANNING_TIME)
        results = super(BenchmarkSession, self)._run_problem_set()

        with open(ROS_PKG_PATH+'/benchmark_latest.p', 'wb') as fp:    # Dump as latest benchmark
            pickle.dump(results, fp)
