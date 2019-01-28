#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Monday, January 28th 2019, 5:18:33 pm
###

import sys
import os
import numpy as np

import rospkg
import rospy

from bayes_opt import BayesianOptimization

from session import Session


ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/scripts/modules'


class BayesOptSession(Session):
    """
    BayesOpt Session
    """

    def __init__(self):
        super(BayesOptSession, self).__init__()
        if self.path_tune:
            rospy.loginfo('Initialising BayesOpt session in %s mode from %s to %s',
                          self.mode, self.start_pose, self.target_pose)
        else:
            rospy.loginfo(
                'Initialising BayesOpt session in %s mode on full problem set', self.mode)

    def run(self):
        pass


def black_box_function(x, y):
    """Function with unknown internals we wish to maximize.

    This is just serving as an example, for all intents and
    purposes think of the internals of this function, i.e.: the process
    which generates its output values, as unknown.
    """
    return (-x ** 2 - (y - 1) ** 2 + 1)


# Bounded region of parameter space
pbounds = {'x': (2, 4), 'y': (-3, 3)}

optimizer = BayesianOptimization(
    f=black_box_function,
    pbounds=pbounds,
    random_state=1,
)

print(optimizer.max)
