#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 2:18:59 pm
# Author: Charlene Leong
# Last Modified: Monday, January 28th 2019, 5:01:28 pm
# Author: Charlene Leong (charleneleong84@gmail.com)
###

import cPickle as pickle

import rospkg
import rospy

from session import Session

ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/scripts'


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

    def run(self):
        headers = ['planner', 'scene', 'query', 'start_pose', 'target_pose', 'avg_runs', 'avg_run_time',
                             'avg_plan_time', 'avg_dist', 'avg_path_length', 'avg_success', 'params']
        super(BenchmarkSession, self)._write_headers(headers=headers, results_path=self.results_path)

        results = {}
        for p in self.planners:
            result = super(BenchmarkSession, self).run_problem_set(
                planner_id=p, save=True, results_path=self.results_path)
            results[p] = result

        with open(ROS_PKG_PATH+'/'+self.planner_select+'_'+self.mode+'.p', 'wb') as f:
            pickle.dump(results, f)

        print('\n')
        rospy.loginfo('Saved results to %s', self.results_path)
        print('\n')
