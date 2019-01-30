#!/usr/bin/env python
###
# File Created: Monday, January 21st 2019, 10:55:57 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Last Modified: Wednesday, January 30th 2019, 10:29:45 am
# Modified By: Charlene Leong
###
import sys
import signal
from timeit import default_timer as timer

import rospy

import moveit_commander

from session import Session


class SMACRun(Session):
    """
    SMACRun Session
    _smac_obj(params): Adds non tunable params from SMAC Session to params_set and executes obj
    """

    def __init__(self, scene, planner):
        super(SMACRun, self).__init__()
        self.planners = [planner]
        self.planner = planner
        self.scenes = [scene]
        self.max_trials = 1  
        self.max_runtime = "None"

    def _smac_obj(self, params):
        # Adding non-tunable params back to params_set
        params_set = dict(self.planner_config[self.planner].items())
        for k, v in params_set.iteritems():
            if not isinstance(v, list):
                params['params_set'][k] = v
        return super(SMACRun, self)._objective(params)


def set_session_params():
    # Porting params from SMAC session
    session_params = rospy.get_param('/parameter_tuning')
    for k, v in session_params.iteritems():
        rospy.set_param('~'+k, v)


if __name__ == '__main__':
    rospy.init_node('smac_run', anonymous=True, log_level=rospy.FATAL)
    set_session_params()

    # SMAC calls the target algo in the following format
    # <algo> <algo params> <instance> <instance specific> <cutoff time> <runlength> <seed> <pcs params>
    planner = sys.argv[1]
    scene = sys.argv[2]
    max_trials = sys.argv[3]

    instance = sys.argv[4]
    specifics = sys.argv[5]
    cutoff = int(float(sys.argv[6]) + 1)
    runlength = int(sys.argv[7])
    seed = str(sys.argv[8])

    # Build a dictionary mapping param k to param v
    pcs_params = sys.argv[9:]
    params_set = dict((name[1:], value) for name, value in zip(pcs_params[::2], pcs_params[1::2]))

    smac_run = SMACRun(scene, planner)

    start_time = timer()
    params = {'planner': planner,
              'params_set': params_set, 'start_time': start_time}

    max_runtime = rospy.get_param('~max_runtime')
    if(max_runtime != "None"):
        params['end_time'] = timer() + int(max_runtime)

    result = smac_run._smac_obj(params)

    quality = result['loss']

    # Result for SMAC: <STATUS>, <runtime>, <runlength>, <quality>, <seed>, <instance-specifics>
    print('Result for SMAC: SUCCESS, ' + str(result['elapsed_time']) + ', 0, ' + str(quality) + ', 0')
