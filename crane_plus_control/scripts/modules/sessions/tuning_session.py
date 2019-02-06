#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 7:18:59 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Wednesday, February 6th 2019, 5:28:49 pm
###

import sys
import os
import datetime
import csv
import pprint
import cPickle as pickle

from timeit import default_timer as timer
from collections import OrderedDict
import pandas as pd
import numpy as np

import rospy

import moveit_commander
import moveit_msgs.msg

from session import Session


class TuningSession(Session):
    """
    Tuning Session Base Class
    """

    def __init__(self):
        super(TuningSession, self).__init__()

        self.n_trial = 0
        self.MAX_RUNTIME = rospy.get_param('~max_runtime')
        
        if self.MODE not in ['default', 'ompl']:
            self.MAX_TRIALS = rospy.get_param('~max_trials')
            if self.MAX_RUNTIME != 'None':
                self.MAX_RUNTIME = int(self.MAX_RUNTIME)
                self.MAX_TRIALS = 10000     # Set to arbitrary large number
                
        self.START_POSE = rospy.get_param('~start_pose')
        self.TARGET_POSE = rospy.get_param('~target_pose')
        self.PATH_TUNE = False
        if (self.START_POSE != 'None') and (self.TARGET_POSE != 'None'):
            self.PATH_TUNE = True

         
    def _load_search_space(self, params_set, *args, **kwargs):
        raise NotImplementedError

    def run_session(self):
        raise NotImplementedError

    def _objective(self, params):
        self.n_trial += 1
        planner = params['planner']
        params_set = params['params_set']
        self.planner_config_obj.set_planner_params(planner, params_set)

        if self.PATH_TUNE:
            results = self._get_stats(self.START_POSE, self.TARGET_POSE)
            stats = {'t_avg_run_time': results['avg_run_time'], 't_avg_plan_time': results['avg_plan_time'],
                     't_avg_dist': results['avg_dist'], 't_avg_path_length': results['avg_path_length'],
                     't_avg_success': results['avg_success']}
        else:
            results, stats = self._run_problem_set(planner_id=planner)

        loss = self._calc_loss(stats)

        current = timer()
        elapsed_time = (current - params['start_time'])

        rospy.loginfo('n_trial: %d elapsed_time(s): %.4f loss: %.4f t_avg_run_time: %.4f t_avg_plan_time: %.4f t_avg_dist: %.4f t_avg_path_length: %.4f t_avg_success: %.4f\n',
                        self.n_trial, elapsed_time, loss, stats['t_avg_run_time'], 
                        stats['t_avg_plan_time'], stats['t_avg_dist'], stats['t_avg_path_length'], 
                        stats['t_avg_success'])

        result = OrderedDict([('n_trial', self.n_trial), ('elapsed_time', elapsed_time), ('loss', loss), 
                              ('planner', planner), ('avg_runs', self.AVG_RUNS), ('t_avg_run_time', 
                              stats['t_avg_run_time']), ('t_avg_plan_time', stats['t_avg_plan_time']),
                              ('t_avg_dist', stats['t_avg_dist']), ('t_avg_path_length', stats['t_avg_path_length']),
                              ('t_avg_success', stats['t_avg_success'])])
        # Save params as str for csv export
        result_csv = OrderedDict(list(result.items())+[('params', str(params_set))])
        result = OrderedDict(list(result.items())+[('params', params_set)])
        # print(json.dumps(result_csv, indent=4))     # Print OrderedDict nicely

        result_df = pd.DataFrame(dict(result_csv), columns=result.keys(), index=[0])

        with open(self.RESULTS_PATH, 'a') as f:
            result_df.to_csv(f, header=False, index=False)

        if (self.MAX_RUNTIME != 'None') and (timer() > params['end_time']):
            rospy.loginfo('Program has run for allotted time (%d secs)\n', self.MAX_RUNTIME)
            rospy.loginfo('Saved results to %s\n', self.RESULTS_PATH)
            sys.exit(0)

        return dict(result)

    def _calc_loss(self,  stats):
        # Plantime is our metric
        success_rate = (1 - stats['t_avg_success']) # Want to max success
        # Returns max plantime if success = 0, otherwise returns plantime
        loss = (stats['t_avg_plan_time']*stats['t_avg_success']) + (self.MAX_PLANTIME*success_rate)

        return loss

