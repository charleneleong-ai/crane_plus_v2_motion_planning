#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 2:50:24 pm
###
import sys
from timeit import default_timer as timer
import pprint

import rospy

from skopt import gp_minimize, forest_minimize, gbrt_minimize
from skopt.space import Real, Categorical

from session import Session


class SKOptSession(Session):
    """
    SKOpt Session 
    """

    def __init__(self):
        super(SKOptSession, self).__init__()
        if self.path_tune:
            rospy.loginfo('Initialising SKOpt session in %s mode from %s to %s',
                          self.mode, self.start_pose, self.target_pose)
        else:
            rospy.loginfo(
                'Initialising SKOpt session in %s mode on full problem set', self.mode)
        self.planner = self.planners[0]
        if self.max_trials < 10:
            rospy.logerr('Max trials must be >= 10 for %s mode\n', self.mode)
            sys.exit(1)

    def _skopt_obj(self, search_space):
        # SKopt obj requires list of params and return scalar loss
        # Converting to be compatible with our obj function
        params = {'planner': self.planner, 'start_time': self.start_time}
        if(self.max_runtime != 'None'):
                            params['end_time'] = timer() + self.max_runtime

        params_set = {}
        for idx, k in enumerate(self.planner_config[self.planner].keys()):
            params_set[k] = search_space[idx]

        params['params_set'] = params_set
        results = super(SKOptSession, self)._objective(params)

        return results['loss']

    def _load_search_space(self, params_set, *args, **kwargs):
        search_space = []
        for k, v in params_set.iteritems():
            if isinstance(v, list):
                search_space.append(Real(v[0], v[1], name=k))
            else:
                search_space.append(Categorical([v], name=k))
        return search_space

    def run_session(self):
        super(SKOptSession, self)._write_headers(self.results_path)

        for planner, params_set in self.planner_config.iteritems():
            search_space = self._load_search_space(params_set)
            
            if(self.max_runtime != 'None'):
                print('\n')
                rospy.loginfo('Executing %s on %s for %d secs',
                                self.mode, planner, self.max_runtime)
            else:
                print('\n')
                rospy.loginfo('Executing %s on %s for %d trials',
                                self.mode, planner, self.max_trials)

            self.n_trial = 0        # Reset to n_trials to zero for each planner        
            self.planner = planner  # Keeping track of current planner
            self.start_time = timer()   # Keeping track of start_time
            if self.mode == 'gp':
                result = gp_minimize(self._skopt_obj, search_space, n_calls=self.max_trials, random_state=0,
                                acq_func='gp_hedge')
                # gp_hedge means probabilistically choose betwn LCB, EI and PI acquisition functions at every iteration
            elif self.mode == 'rf':
                result = forest_minimize(self._skopt_obj, search_space, n_calls=self.max_trials, random_state=0,
                                base_estimator='RF', acq_func='EI')
            elif self.mode == 'et':
                result = forest_minimize(self._skopt_obj, search_space, n_calls=self.max_trials, random_state=0,
                                base_estimator='ET', acq_func='EI')
            elif self.mode == 'gbrt':
                result = gbrt_minimize(self._skopt_obj, search_space, n_calls=self.max_trials, random_state=0,
                                acq_func='EI')

        print('\n')
        rospy.loginfo('Saved results to %s\n', self.results_path)
