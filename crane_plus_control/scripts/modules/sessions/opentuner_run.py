#!/usr/bin/env python
###
# File Created: Tuesday, 29th January 2019 4:20:52 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Wednesday, February 6th 2019, 4:06:08 pm
###

import argparse
from timeit import default_timer as timer

import rospy

import opentuner
from opentuner.api import TuningRunManager
from opentuner.measurement.interface import DefaultMeasurementInterface
from opentuner import ConfigurationManipulator
from opentuner import FloatParameter
from opentuner import Result

from tuning_session import TuningSession


class OpenTunerRun(TuningSession):
    """
    OpenTunerRun Session
    _opentuner_obj(params): Adds non tunable params from OpenTuner Session to params_set and executes obj
    """

    def __init__(self, args):
        super(OpenTunerRun, self).__init__()
        self.args = args
        self.planners = [self.args.planner]
        self.planner = self.args.planner

    def _opentuner_obj(self, params):
        # Adding non-tunable params back to params_set
        params_set = dict(self.planner_config[self.planner].items())
        for k, v in params_set.iteritems():
            if not isinstance(v, list):
                params['params_set'][k] = v
        return super(OpenTunerRun, self)._objective(params)

    def _load_search_space(self, params_set, *args, **kwargs):
        # Configuration manipulator holds the search space
        manipulator = ConfigurationManipulator()
        for k, v in params_set.iteritems():
            if isinstance(v, list):
                manipulator.add_parameter(FloatParameter(k, v[0], v[1]))
        return manipulator

    def run_session(self):
        params_set = dict(self.planner_config[self.planner].items())
        manipulator = self._load_search_space(params_set)
        interface = DefaultMeasurementInterface(args=self.args,
                                                manipulator=manipulator)
        # Using OpenTuner API
        # https://github.com/jansel/opentuner/blob/master/examples/py_api/api_example.py
        api = TuningRunManager(interface, self.args)

        self.n_trial = 0        # Reset to n_trials to zero for each planner
        start_time = timer()
        params = {'planner': self.planner, 'start_time': start_time}
        if(self.MAX_RUNTIME != 'None'):
            params['end_time'] = timer() + self.MAX_RUNTIME
            print('\n')
            rospy.loginfo('Executing %s on %s for %d secs',
                          self.MODE, params['planner'], self.MAX_RUNTIME)
        else:
            print('\n')
            rospy.loginfo('Executing %s on %s for %d trials',
                          self.MODE, params['planner'], self.MAX_TRIALS)

        for _ in range(self.MAX_TRIALS):
            desired_result = api.get_next_desired_result()
            params_set = desired_result.configuration.data
            params['params_set'] = params_set

            run_result = self._opentuner_obj(params)
            result = Result(time=run_result['loss'])
            api.report_result(desired_result, result)


def set_session_params():
    # Porting params from OpenTuner session
    session_params = rospy.get_param('/parameter_tuning')
    for k, v in session_params.iteritems():
        rospy.set_param('~'+k, v)


if __name__ == "__main__":
    rospy.init_node('opentuner_run', anonymous=True)
    set_session_params()

    parser = argparse.ArgumentParser(parents=opentuner.argparsers())
    parser.add_argument('--planner', type=str, default='')
    args = parser.parse_args()
    opentuner_run = OpenTunerRun(args)
    opentuner_run.run_session()
