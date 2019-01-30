#!/usr/bin/env python
###
# File Created: Saturday, January 12th 2019, 11:23:55 am
# Author: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 2:56:46 pm
# Modified By: Charlene Leong
###

import sys
import rospy
import moveit_commander
from modules.hyperopt_session import HyperOptSession
from modules.benchmark_session import BenchmarkSession
from modules.smac_session import SMACSession
from modules.opentuner_session import OpenTunerSession
from modules.skopt_session import SKOptSession

def check_params(mode):
    """Checks for valid ROS parameters
    
    Args:
        mode (str): Mode of parameter tuning session
    """
    modes = ['default', 'tpe', 'rand', 'ompl', 'smac', 'auc_bandit', 'gp', 'rf']
    if mode not in modes:
        rospy.logerr('Invalid mode.')
        rospy.logerr('Please choose from %s', str(modes))
        sys.exit(1)

    planner_select = rospy.get_param('~planner_select')
    if planner_select not in ['Cano_etal', 'Burger_etal']:
        rospy.logerr('Invalid planner config select.')
        rospy.logerr('Please choose from %s', str(['Cano_etal', 'Burger_etal']))
        sys.exit(1)

    max_runtime = rospy.get_param('~max_runtime')
    if max_runtime != 'None':
        try:
            max_runtime = int(max_runtime)
        except ValueError:
            rospy.logerr("Not an valid int value. Please express max_runtime in secs.")
            sys.exit(1)

    start_pose = rospy.get_param('~start_pose')
    target_pose = rospy.get_param('~target_pose')
    named_states = rospy.get_param('~named_states')

    if (start_pose  == "None") and (target_pose  == "None"):
        return
    # Will also return error is only start or only target is None
    elif start_pose not in named_states:    
        rospy.logerr('start_pose not in list of named_states')
        rospy.logerr('Please choose from %s', str(named_states))
        sys.exit(1)
    elif target_pose not in named_states:
        rospy.logerr('target_pose not in list of named_states')
        rospy.logerr('Please choose from %s', str(named_states))
        sys.exit(1)
    

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning', anonymous=True)

    mode = rospy.get_param('~mode')
    check_params(mode)

    if(mode in ['default', 'ompl']):
        session = BenchmarkSession()
    elif(mode in ['tpe', 'rand']):
        session = HyperOptSession()
    elif(mode == 'smac'):
        session = SMACSession()
    elif(mode == 'auc_bandit'):
        session = OpenTunerSession()
    elif(mode == 'gp'):
        session = SKOptSession()

    session.run_session()

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
