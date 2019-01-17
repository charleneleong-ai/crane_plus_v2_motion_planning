#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 1:56:33 pm
# Modified By: Charlene Leong
# Last Modified: Thursday, January 17th 2019, 1:31:30 pm
# Author: Charlene Leong (charleneleong84@gmail.com)
###

import sys
import rospy
import moveit_commander
from modules.hyperopt_session import HyperOptSession
from modules.benchmark_session import BenchmarkSession

def check_params(mode):
    if mode not in ['baseline', 'tpe', 'rand', 'ompl']:
        rospy.logerr('Invalid mode.')
        sys.exit(1)

    planner_select = rospy.get_param('~planner_config')
    if planner_select not in ['Cano_etal']:
        rospy.logerr('Invalid planner config select')
        sys.exit(1)
    
    start_pose = rospy.get_param('~start_pose')
    target_pose = rospy.get_param('~target_pose')
    named_states = rospy.get_param('~named_states')

    if target_pose not in named_states:
        rospy.logerr('target_pose not in list of named_states')
        rospy.logerr(named_states)
        sys.exit(1)
    elif start_pose not in named_states:
        rospy.logerr('start_pose not in list of named_states')
        rospy.logerr(named_states)
        sys.exit(1)


def main():
    
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning', anonymous=True)

    mode = rospy.get_param('~mode')
    check_params(mode)

    if(mode in ['baseline', 'ompl']):
        session = BenchmarkSession(mode)
    elif(mode in ['tpe', 'rand']):
        session = HyperOptSession(mode)

    # session.run()
    # session.get_results()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
