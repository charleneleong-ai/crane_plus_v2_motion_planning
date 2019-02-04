#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 2:18:59 pm
# Author: Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 11:02:12 am
###

import rospy
import csv
from session import Session


class BenchmarkSession(Session):
    """
    Benchmarking Sessions 
    run(): Runs in ompl and default mode
    """

    def __init__(self):
        super(BenchmarkSession, self).__init__()
        if self.PATH_TUNE:
            rospy.loginfo('Initialising benchmarking session in %s mode from %s to %s\n',
                          self.MODE, self.START_POSE, self.TARGET_POSE)
        else:
            rospy.loginfo('Initialising benchmarking session in %s mode on full problem set\n', 
                          self.MODE)

    def run_session(self):
        super(BenchmarkSession, self)._write_headers(path=self.RESULTS_PATH)

        results = {}
        for planner, params_set in self.planner_config.iteritems():
            rospy.loginfo('Executing %s on %s', self.MODE, planner)
            result, stats = super(BenchmarkSession, self)._run_problem_set(planner_id=planner)
            results[planner] = result       

            rospy.loginfo('planner: %s avg_runs: %d t_avg_run_time: %.4f t_avg_plan_time: %.4f t_avg_dist: %.4f t_avg_path_length: %.4f t_avg_success: %.4f\n',
                        planner, self.AVG_RUNS, stats['t_avg_run_time'], 
                        stats['t_avg_plan_time'], stats['t_avg_dist'], stats['t_avg_path_length'], 
                        stats['t_avg_success'])
                        
            with open(self.RESULTS_PATH, 'a') as f:
                writer = csv.writer(f)
                writer.writerow([planner, self.AVG_RUNS, stats['t_avg_run_time'], stats['t_avg_plan_time'],
                                 stats['t_avg_dist'], stats['t_avg_path_length'], stats['t_avg_success'],
                                 params_set])

            

        super(BenchmarkSession, self)._dump_results(results)

        print('\n')
        rospy.loginfo('Saved results to %s\n', self.RESULTS_PATH)
