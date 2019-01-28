#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 7:18:59 pm
# Author: Charlene Leong
# Last Modified: Monday, January 28th 2019, 10:38:02 am
# Modified By: Charlene Leong
###

import sys
import time
import cPickle as pickle
from functools import partial

import csv
import json
import pprint
import pandas as pd

import rospkg
import rospy

from hyperopt import hp, rand, tpe, Trials, fmin

from session import Session

ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/scripts'


class HyperOptSession(Session):
    """
    Hyperopt Session runs in rand and tpe mode
    """
    def __init__(self):
        super(HyperOptSession, self).__init__()
        if self.path_tune:
            rospy.loginfo('Initialising hyperopt session in %s mode from %s to %s',
                          self.mode, self.start_pose, self.target_pose)
        else:
            rospy.loginfo('Initialising hyperopt session in %s mode on full problem set', self.mode)

    def _hpt_obj(self, params):
        return super(HyperOptSession, self)._objective(params)

    def run(self):
        self.group.set_planning_time(self.max_plan_time)
        with open(self.results_path, 'w') as f:     # Write headers
            writer = csv.writer(f)
            writer.writerow(['elapsed_time', 'n_trial', 'loss', 'planner', 'avg_runs', 't_avg_run_time',
                             't_avg_plan_time', 't_avg_dist', 't_avg_path_length', 't_avg_success', 'params'])

        start_time = time.time()
        # Setting up the parameter search space and parameters
        for planner, params_set in self.planner_config.iteritems():
            params_set = dict(self.planner_config[planner].items())
            for k, v in params_set.iteritems():
                if isinstance(v, list):
                    # Discrete uniform dist [begin_range, end_range, step]
                    params_set[k] = hp.quniform(k, v[0], v[1], v[2])

            params = {'planner': planner, 'params_set': params_set,
                          'start_time': start_time}
                    
            if(self.max_runtime != "None"):
                params['end_time'] = time.time() + self.max_runtime
                print('\n')
                rospy.loginfo('Executing %s on %s for: %d secs',
                              self.mode, params['planner'], self.max_runtime)
            else:
                print('\n')
                rospy.loginfo('Executing %s on %s for %d trials',
                              self.mode, params['planner'], self.max_trials)

            self.n_trial = 0        # Reset to n_trials to zero for each planner
            if self.mode == 'tpe':
                algo = partial(tpe.suggest,
                               # Sample 100 candidates and select candidate that has highest Expected Improvement (EI)
                               n_EI_candidates=100,
                               # Use 20% of best observations to estimate next set of parameters
                               gamma=0.2,
                               # First 20 trials are going to be random
                               n_startup_jobs=20)
            elif self.mode == 'rand':
                algo = rand.suggest
                
            trials = Trials()
            best = fmin(fn=self._hpt_obj, space=params, algo=algo,
                        max_evals=self.max_trials, trials=trials)

        with open(ROS_PKG_PATH+'/'+self.planner_select+'_'+self.mode+'.p', 'wb') as f:
            pickle.dump(trials, f)

        print('\n')
        rospy.loginfo('Saved results to %s', self.results_path)
        print('\n')

# class RunningMean(object):
#     def __init__(self, t=0.0, n=0):
#         self.t=t
#         self.n = n

#     def __iadd__(self, other):
#         self.t += other
#         self.n += 1
#         return self

#     def mean(self):
#         return (self.t/self.n if self.n else 0)

#     def __repr__(self):
#         return "RunningMean(t=%f, n=%i)" %(self.t, self.n)
