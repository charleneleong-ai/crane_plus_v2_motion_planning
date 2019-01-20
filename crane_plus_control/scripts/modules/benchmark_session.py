#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 2:18:59 pm
# Author: Charlene Leong
# Last Modified: Monday, January 21st 2019, 8:38:33 am
# Author: Charlene Leong (charleneleong84@gmail.com)
###

import sys
import numpy
import cPickle as pickle
import pprint
import csv
import itertools

import rospkg
import rospy
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

        # if not os.path.isfile(results_path):    # If file doesnt exist write header
        with open(self.results_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['planner', 'scene', 'query', 'start_pose', 'target_pose', 'avg_runs', 'avg_run_time',
                             'avg_plan_time', 'avg_dist', 'avg_path_length', 'avg_success', 'params'])

        results = {}
        for p in self.planners:
            result = super(BenchmarkSession, self)._run_problem_set(
                planner_id=p, save=True, results_path=self.results_path)
            results[p] = result

        # pprint.pprint(results)

        # Dump as latest benchmark
        with open(ROS_PKG_PATH+'/'+self.planner_config_obj.planner_select+'_'+self.mode+'.p', 'wb') as f:
            pickle.dump(results, f)

        print("\n")
        rospy.loginfo('Saved results to %s', self.results_path)
        print("\n")
