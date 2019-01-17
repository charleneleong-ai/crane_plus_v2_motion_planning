#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 10:02:24 am
# Modified By: Charlene Leong
# Last Modified: Thursday, January 17th 2019, 7:43:20 pm
# Author: Charlene Leong (charleneleong84@gmail.com)
###

import sys
from timeit import default_timer as timer
from collections import OrderedDict, defaultdict
import itertools

import csv
import json
import pprint
import pandas as pd
import numpy as np
from functools import partial

import rospkg
import rospy
import std_msgs.msg

import moveit_commander
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
import moveit_msgs.msg

from hyperopt import hp, rand, tpe, Trials, fmin, STATUS_OK
from hyperopt.pyll.stochastic import sample

from session import Session


class HyperOptSession(Session):
    """Constructor for parameter tuning session
    
    Args:
        Session (object): Session object initiates session with default funcitons
    """
    PLANNING_TIME = 3  # seconds

    def __init__(self, mode):
        super(HyperOptSession, self).__init__(mode)
        rospy.loginfo('Initialising hyperopt parameter tuning session in %s mode', mode)

 
    def _objective(self, params):
        self.n_trial += 1
        # Extract param set
        planner = params['planner']
        params_set = params['params_set']

        # Set new params
        self.planner_config_obj.set_planner_params(planner, params_set)

        # Execute experiment for iter times and get planning and run_time stats
        results, stats = super(HyperOptSession, self)._run_problem_set(planner_id=planner)

        # loss = sum(stats.values())
        loss = stats['t_avg_path_length']

        rospy.loginfo('n_trial: %d loss: %.4f t_avg_run_time: %.4f t_avg_plan_time: %.4f t_avg_dist: %.4f t_avg_path_length: %.4f',
                      self.n_trial, loss, stats['t_avg_run_time'], stats['t_avg_plan_time'], stats['t_avg_dist'], stats['t_avg_path_length'])

        # # Create OrderedDict to write to CSV
        result = OrderedDict([('n_trial', self.n_trial), ('loss', loss), ('planner', 'planner'), ('avg_runs', self.iter), 
                             ('t_avg_run_time', stats['t_avg_run_time']), ('t_avg_plan_time', stats['t_avg_dist']),
                             ('t_avg_dist', stats['t_avg_dist']), ('t_avg_path_length', stats['t_avg_path_length'])])
        # Need to save params as str for csv
        result_csv = OrderedDict(list(result.items()) + [('params', str(params_set))])
        result = OrderedDict(list(result.items()) + [('params', params_set), ('status', STATUS_OK)])
        # print(json.dumps(result_csv, indent=4))     # Print OrderedDict nicely

        result_df = pd.DataFrame(dict(result_csv), columns=result.keys(), index=[0])
        with open(self.results_path, 'a') as f:
            result_df.to_csv(f, header=False, index=False)

        return dict(result)

    def run(self):
        self.group.set_planning_time(self.PLANNING_TIME)

        with open(self.results_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['n_trial', 'loss', 'planner', 'avg_runs', 't_avg_run_time',
                             't_avg_plan_time', 't_avg_dist', 't_avg_path_length', 'params'])

        # Setting up the parameter search space and parameters
        for planner, params_set in self.planner_config.iteritems():
            params_set = dict(self.planner_config[planner].items())
            for k, v in params_set.iteritems():
                if isinstance(v, list):
                    # Discrete uniform dist [begin_range, end_range, step]
                    params_set[k] = hp.quniform(k, v[0], v[1], v[2])
            params = {'planner': planner, 'params_set': params_set}

            print('\n')
            rospy.loginfo('Executing %s on %s:  Max trials: %d ', self.mode,
                          params['planner'], self.max_trials)
            self.n_trial = 0        # Reset to n_trials to zero for each planner
            if self.mode == 'tpe':
                algo = partial(tpe.suggest,
                               # Sample 1000 candidate and select candidate that has highest Expected Improvement (EI)
                               n_EI_candidates=100,
                               # Use 20% of best observations to estimate next set of parameters
                               gamma=0.2,
                               # First 20 trials are going to be random
                               n_startup_jobs=20)
            elif self.mode == 'rand':
                algo = rand.suggest

            trials = Trials()
            best = fmin(fn=self._objective, space=params, algo=algo,
                        max_evals=self.max_trials, trials=trials)

        print("\n")
        rospy.loginfo('Saved results to %s', self.results_path)
        print("\n")

class RunningMean(object):
    def __init__(self, t=0.0, n=0):
        self.t=t
        self.n = n

    def __iadd__(self, other):
        self.t += other
        self.n += 1
        return self

    def mean(self): 
        return (self.t/self.n if self.n else 0)

    def __repr__(self):
        return "RunningMean(t=%f, n=%i)" %(self.t, self.n)
