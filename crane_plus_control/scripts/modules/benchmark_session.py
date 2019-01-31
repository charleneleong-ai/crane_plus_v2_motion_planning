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
        if self.path_tune:
            rospy.loginfo('Initialising benchmarking session in %s mode from %s to %s',
                          self.mode, self.start_pose, self.target_pose)
        else:
            rospy.loginfo(
                'Initialising benchmarking session in %s mode on full problem set', self.mode)

    def run_session(self):
        super(BenchmarkSession, self)._write_headers(path=self.results_path)

        results = {}
        for p in self.planners:
            result, stats = super(BenchmarkSession, self)._run_problem_set(planner_id=p, save=True)
            results[p] = result

            with open(self.results_path, 'a') as f:
                writer = csv.writer(f)
                writer.writerow(p, self.avg_runs, stats['t_avg_run_time'], stats['t_avg_plan_time'],
                                stats['t_avg_dist'], stats['t_avg_path_length'], stats['t_avg_success'], 
                                stats['params'])

        super(BenchmarkSession, self)._dump_results(results)

        print('\n')
        rospy.loginfo('Saved results to %s\n', self.results_path)
