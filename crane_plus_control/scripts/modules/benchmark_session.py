#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 2:18:59 pm
# Author: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 10:41:38 am
# Author: Charlene Leong (charleneleong84@gmail.com)
###

import rospy

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
            rospy.loginfo('Initialising benchmarking session in %s mode on full problem set', self.mode)

    def run_session(self):
        super(BenchmarkSession, self)._write_headers(self.results_path)

        results = {}
        for p in self.planners:
            result = super(BenchmarkSession, self)._run_problem_set(
                planner_id=p, save=True, results_path=self.results_path)
            results[p] = result

        super(BenchmarkSession, self)._dump_results(results)

        print('\n')
        rospy.loginfo('Saved results to %s\n', self.results_path)