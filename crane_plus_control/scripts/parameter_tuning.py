#!/usr/bin/env python
###
# File Created: Saturday, January 12th 2019, 11:23:55 am
# Author: Charlene Leong
# Last Modified: Sunday, January 20th 2019, 12:13:13 am
# Modified By: Charlene Leong
###

import sys
import rospy
import moveit_commander
from modules.hyperopt_session import HyperOptSession
from modules.benchmark_session import BenchmarkSession


def check_params(mode):
    if mode not in ['default', 'tpe', 'rand', 'ompl']:
        rospy.logerr('Invalid mode.')
        rospy.logerr('Please choose from %s', str(
            ['default', 'tpe', 'rand', 'ompl']))
        sys.exit(1)

    planner_select = rospy.get_param('~planner_config')
    if planner_select not in ['Cano_etal']:
        rospy.logerr('Invalid planner config select.')
        rospy.logerr('Please choose from %s', str(['Cano_etal']))
        sys.exit(1)

    start_pose = rospy.get_param('~start_pose')
    target_pose = rospy.get_param('~target_pose')
    named_states = rospy.get_param('~named_states')

    if target_pose not in named_states:
        rospy.logerr('target_pose not in list of named_states')
        rospy.logerr('Please choose from %s', str(named_states))
        sys.exit(1)
    elif start_pose not in named_states:
        rospy.logerr('start_pose not in list of named_states')
        rospy.logerr('Please choose from %s', str(named_states))
        sys.exit(1)


def main():

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning', anonymous=True)

    mode = rospy.get_param('~mode')
    check_params(mode)

    if(mode in ['default', 'ompl']):
        session = BenchmarkSession(mode)
    elif(mode in ['tpe', 'rand']):
        session = HyperOptSession(mode)

    session.run()
    # session.get_results()
    moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
