#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Monday, January 28th 2019, 6:51:24 pm
###

import sys
import os
import numpy as np

import rospkg
import rospy

from skopt import gp_minimize
from skopt.space import Real, Integer
from skopt.utils import use_named_args

from session import Session


ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/scripts/modules'


class SKOptSession(Session):
    """
    SKOpt Session 
    """

    def __init__(self):
        super(SKOptSession, self).__init__()
        if self.path_tune:
            rospy.loginfo('Initialising BayesOpt session in %s mode from %s to %s',
                          self.mode, self.start_pose, self.target_pose)
        else:
            rospy.loginfo(
                'Initialising BayesOpt session in %s mode on full problem set', self.mode)

    def _bayesopt_obj(self, params):

        return super(SKOptSession, self)._objective(params)

    def run(self):
        headers = ['elapsed_time', 'n_trial', 'loss', 'planner', 'avg_runs', 't_avg_run_time',
                             't_avg_plan_time', 't_avg_dist', 't_avg_path_length', 't_avg_success', 'params']
        super(SKOptSession, self)._write_headers(headers=headers, results_path=self.results_path)

        # best = gp_minimize(fn=_bayesopt_obj, )
