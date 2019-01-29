#!/usr/bin/env python
###
# File Created: Tuesday, 29th January 2019 4:20:52 pm
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 8:59:30 am
###
#!/usr/bin/env python
###
# File Created: Tuesday, 29th January 2019 11:29:40 am
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Tuesday, January 29th 2019, 4:20:23 pm
###
import sys
import argparse
from timeit import default_timer as timer

import numpy as np
import rospkg
import rospy

import moveit_commander

import opentuner
from opentuner.utils import adddeps
from opentuner.api import TuningRunManager
from opentuner.measurement.interface import DefaultMeasurementInterface
from opentuner import ConfigurationManipulator
from opentuner import FloatParameter
from opentuner.search.manipulator import FloatArray
from opentuner import MeasurementInterface
from opentuner import Result

from session import Session


# parser.add_argument('--domain', type=float, default=1000)
# parser.add_argument('--function', default='rosenbrock')

class OpenTunerRun(Session):
    """
    OpenTunerRun Session
    _smac_obj(params): Adds non tunable params from SMAC Session to params and executes the problem set
    """

    def __init__(self, args):
        super(OpenTunerRun, self).__init__()
        self.args = args
        self.planners = [self.args.planner]
        self.planner = self.args.planner

    def _opentuner_obj(self, params):
        params_set = dict(self.planner_config[self.planner].items())
        for k, v in params_set.iteritems():
            if not isinstance(v, list):
                params['params_set'][k] = v
        return super(OpenTunerRun, self)._objective(params)

    def _load_search_space(self, planner, params_set, *args, **kwargs):
        manipulator = ConfigurationManipulator()

        params_set = dict(self.planner_config[planner].items())
        for k, v in params_set.iteritems():
            if isinstance(v, list):
                if self.planner_select == 'Cano_etal':
                    # Difficulty defining discrete search space
                    # param_array = np.arange(v[0], v[1]+v[2], v[2]) 
                    # print(k, len(param_array), type(param_array[0]), param_array)
                    # manipulator.add_parameter(FloatArray(k, len(param_array), v[1], v[0]))
                    manipulator.add_parameter(FloatParameter(k, v[0], v[1]))
                elif self.planner_select == 'Burger_etal':
                    manipulator.add_parameter(FloatParameter(k, v[0], v[1]))

        return manipulator

    def run_session(self):
        params_set = dict(self.planner_config[self.planner].items())
        manipulator = self._load_search_space(self.planner, params_set)
        interface = DefaultMeasurementInterface(args=self.args,
                                                manipulator=manipulator)
        api = TuningRunManager(interface, self.args)

        self.n_trial = 0        # Reset to n_trials to zero for each planner
        start_time = timer()
        params = {'planner': self.planner, 'start_time': start_time}
        if(self.max_runtime != 'None'):
            params['end_time'] = timer() + self.max_runtime
            print('\n')
            rospy.loginfo('Executing %s on %s for  %d secs',
                          self.mode, params['planner'], self.max_runtime)
        else:
            print('\n')
            rospy.loginfo('Executing %s on %s for %d trials',
                          self.mode, params['planner'], self.max_trials)

        for _ in range(self.max_trials):
            desired_result = api.get_next_desired_result()
            if desired_result is None:
                # The search space for this example is very small, so sometimes
                # the techniques have trouble finding a config that hasn't already
                # been tested.  Change this to a continue to make it try again.
                break
            params_set = desired_result.configuration.data
            params['params_set'] = params_set

            # This fn breaks if end_time is up
            run_result = self._opentuner_obj(params)
            result = Result(time=run_result['loss'])
            api.report_result(desired_result, result)

        print('\n')
        rospy.loginfo('Saved results to %s', self.results_path)
        print('\n')
        sys.exit(0)


# Porting params from SMAC session
def set_session_params():
    session_params = rospy.get_param('/parameter_tuning')
    for k, v in session_params.iteritems():
        rospy.set_param('~'+k, v)


def sigint_exit(signal, frame):
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('opentuner_run', anonymous=True)

    set_session_params()

    parser = argparse.ArgumentParser(parents=opentuner.argparsers())
    parser.add_argument('--planner', type=str, default='')
    args = parser.parse_args()
    opentuner_run = OpenTunerRun(args)
    opentuner_run.run_session()
