#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 7:18:59 pm
# Author: Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 11:02:12 am
###

from timeit import default_timer as timer
from functools import partial

import rospy

from hyperopt import hp, rand, tpe, Trials, fmin

from session import Session


class HyperOptSession(Session):
    """
    Hyperopt Session
    _hpt_obj(params): Hyperopt obj fn inherited from Session class
    run(): Runs in tpe or rand mode
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

    def _load_search_space(self,  params_set, *args, **kwargs):
        for k, v in params_set.iteritems():
            if isinstance(v, list):
                if self.planner_select == "Cano_etal":
                    # Discrete uniform dist [begin_range, end_range, step]
                    params_set[k] = hp.quniform(k, v[0], v[1], v[2])
                elif self.planner_select == "Burger_etal":
                    params_set[k] = hp.uniform(k, v[0], v[1])
        return params_set

    def run_session(self):
        super(HyperOptSession, self)._write_headers(path=self.results_path)

        # Setting up the parameter search space and parameters
        for planner, params_set in self.planner_config.iteritems():
            params_set = self._load_search_space(params_set)

            start_time = timer()
            params = {'planner': planner, 'params_set': params_set,'start_time': start_time}
                    
            if(self.max_runtime != "None"):
                params['end_time'] = timer() + self.max_runtime
                print('\n')
                rospy.loginfo('Executing %s on %s for %d secs',
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

        super(HyperOptSession, self)._dump_results(trials)
        
        print('\n')
        rospy.loginfo('Saved results to %s\n', self.results_path)
