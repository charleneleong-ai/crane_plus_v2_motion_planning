#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 8:23:06 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By:
# Last Modified:
###


import sys
import os
import csv
from os.path import isfile, join, split, exists
import glob
import ast

import pandas as pd
import pprint

import rospkg
import rospy

import moveit_commander
import moveit_msgs.msg

from utils.ros import check_rosparams
from utils.files import unique

from modules import Session
from modules import PlannerConfig
from modules import Scene


class Result(Session):
    def __init__(self, planner_select, mode, planner):
        super(Result, self).__init__()
        self.PLANNER_SELECT = planner_select
        self.MODE = mode
        self.PLANNER = planner
        self.PLANNER_ID = planner+'kConfigDefault'

        rospy.loginfo(
            'Initialising result session for %s %s in %s mode \n', self.PLANNER_SELECT, self.PLANNER, self.MODE)

        # Overwriting the results path
        self.OUTPUT_DIR = self.ROS_PKG_PATH+'/results/param_tests/'
        if not exists(self.OUTPUT_DIR):
            os.makedirs(self.OUTPUT_DIR)

        self.RESULTS_PATH = self.OUTPUT_DIR+self.PLANNER_SELECT + \
            '_'+self.MODE+'_'+self.PLANNER+'.csv'

    def _set_best_params(self):
        # Return list of all fps in result dir of selected planner select, planner and mode in run folders
        fps = [y for x in os.walk(self.RESULTS_DIR)
                    for y in glob.glob(join(x[0], '*.csv'))
                        if (self.PLANNER_SELECT in y) and 
                            (self.PLANNER in y) and
                            (self.MODE in y) and
                            ('run' in y)]

        # Returns a dict of the best run out of all runs with the min loss
        best_run = {'run': 0, 'min_loss': float('inf')}
        for fp in fps:
            df = pd.read_csv(fp, index_col=False)
            df_best_run = df.iloc[df['loss'].idxmin(axis=1)]

            if df_best_run['loss'] < best_run['min_loss']:
                best_run['min_loss'] = df_best_run['loss']
                best_run['min_run'] = df_best_run.to_dict()
                best_run['run'] = int(fp.split(os.sep)[-2].split('_')[-1])
        
        # Extracts the params set str as dict and sets planner params
        params_set = ast.literal_eval(best_run['min_run']['params'])

        rospy.loginfo("Setting to best params for %s %s %s in run_%d",
                      self.PLANNER_SELECT, self.PLANNER, self.MODE, best_run['run'])
        self.planner_config_obj.set_planner_params(self.PLANNER_ID, params_set)

        return best_run

    def run_param_test(self, save=False):
        best_run = self._set_best_params()
        best_run_stats = best_run['min_run']

        rospy.loginfo('BEST RUN STATS: t_avg_run_time: %.4f t_avg_plan_time: %.4f t_avg_dist: %.4f t_avg_path_length: %.4f t_avg_success: %.4f\n',
                      best_run_stats['t_avg_run_time'], best_run_stats['t_avg_plan_time'], best_run_stats[
                          't_avg_dist'], best_run_stats['t_avg_path_length'],
                      best_run_stats['t_avg_success'])

        results_log, test_stats = self._run_problem_set(self.PLANNER_ID)
        rospy.loginfo('TEST STATS: t_avg_run_time: %.4f t_avg_plan_time: %.4f t_avg_dist: %.4f t_avg_path_length: %.4f t_avg_success: %.4f\n',
                      test_stats['t_avg_run_time'], test_stats['t_avg_plan_time'], test_stats['t_avg_dist'], test_stats['t_avg_path_length'],
                      test_stats['t_avg_success'])

        if save == True:
            cols = ['avg_runs', 't_avg_run_time', 't_avg_plan_time',
                    't_avg_dist', 't_avg_path_length', 't_avg_success', 'params']
            best_run_stats = {k: best_run_stats[k] for k in cols}
            best_run_stats['run'] = 'best_run'

            test_stats['run'] = 'test_run'
            test_stats['avg_runs'] = self.AVG_RUNS
            test_stats['params'] = best_run_stats['params']

            cols = ['run'] + cols
            df_best_run_stats = pd.DataFrame(
                dict(best_run_stats), columns=cols, index=[0])
            df_test_stats = pd.DataFrame(
                dict(test_stats), columns=cols, index=[0])

            df_result = pd.concat([df_best_run_stats, df_test_stats])

            with open(self.RESULTS_PATH, 'w') as f:
                df_result.to_csv(f, index=False)
                rospy.loginfo('Saved results to %s\n', self.RESULTS_PATH)

        return best_run_stats, test_stats

def param_test():
    """
    Scans through all final results and tests the best params for each unique combination 
    of planner select, planner and mode and saves to /results/param_tests/
    """

    ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/results/final'
    # Return list of unique filenames in dir
    filenames = unique(ROS_PKG_PATH+"/*/*.csv")

    planner_select = []
    modes = []
    planners = []
    for fn in filenames:
        if not any(x in fn for x in ['default', 'ompl']):
            fn = fn.split('_')
            planner_select.append(fn[0]+'_'+fn[1])
            modes.append(fn[-2])
            planners.append(fn[-1].split('.')[-2])

    # Performs params test for each unique combination of planner select, mode and planner
    for ps in set(planner_select):
        for m in set(modes):
            for p in set(planners):
                if (ps == 'Cano_etal') and (p == 'BiTRRT'):     # BiTTRT only in Burger_etal
                    continue
                result_session = Result(ps, m, p)
                result_session.run_param_test(save=True)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning', anonymous=True)
    check_rosparams()
    param_test()
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
