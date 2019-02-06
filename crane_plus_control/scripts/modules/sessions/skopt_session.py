#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Wednesday, February 6th 2019, 4:06:51 pm
###
import sys
from timeit import default_timer as timer

import rospy

from skopt import gp_minimize, forest_minimize, gbrt_minimize
from skopt.space import Real, Categorical

from tuning_session import TuningSession


class SKOptSession(TuningSession):
    """
    SKOpt Session 
    """

    def __init__(self):
        super(SKOptSession, self).__init__()
        if self.PATH_TUNE:
            rospy.loginfo('Initialising SKOpt session in %s mode from %s to %s\n',
                          self.MODE, self.START_POSE, self.TARGET_POSE)
        else:
            rospy.loginfo(
                'Initialising SKOpt session in %s mode on full problem set\n', self.MODE)
       
        if self.MAX_TRIALS < 10:
            rospy.logerr('Max trials must be >= 10 for %s mode\n', self.MODE)
            sys.exit(1)

    def _skopt_obj(self, search_space):
        # SKopt obj requires list of params and return scalar loss
        # Converting to be compatible with our obj function
        params = {'planner': self.planner, 'start_time': self.start_time}
        if(self.MAX_RUNTIME != 'None'):
            params['end_time'] = self.end_time

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
        super(SKOptSession, self)._write_headers(path=self.RESULTS_PATH)

        for planner in self.planners:
            params_set = dict(self.planner_config[planner].items())
            search_space = self._load_search_space(params_set)
            
            self.n_trial = 0        # Reset to n_trials to zero for each planner        
            self.planner = planner  # Keeping track of current planner
            self.start_time = timer()   # Keeping track of start_time
            if(self.MAX_RUNTIME != 'None'):
                self.end_time = self.start_time+self.MAX_RUNTIME  # Keeping track of end_time
                rospy.loginfo('Executing %s on %s for %d secs', self.MODE, planner, self.MAX_RUNTIME)            
            else:
                rospy.loginfo('Executing %s on %s for %d trials', self.MODE, planner, self.MAX_TRIALS)

            if self.MODE == 'gp':
                result = gp_minimize(self._skopt_obj, search_space, n_calls=self.MAX_TRIALS, random_state=0,
                                acq_func='gp_hedge')
                # gp_hedge means probabilistically choose betwn LCB, EI and PI acquisition functions at every iteration
            elif self.MODE == 'rf':
                result = forest_minimize(self._skopt_obj, search_space, n_calls=self.MAX_TRIALS, random_state=0,
                                base_estimator='RF', acq_func='EI')
            elif self.MODE == 'et':
                result = forest_minimize(self._skopt_obj, search_space, n_calls=self.MAX_TRIALS, random_state=0,
                                base_estimator='ET', acq_func='EI')
            elif self.MODE == 'gbrt':
                result = gbrt_minimize(self._skopt_obj, search_space, n_calls=self.MAX_TRIALS, random_state=0,
                                acq_func='EI')

        rospy.loginfo('Saved results to %s\n', self.RESULTS_PATH)
