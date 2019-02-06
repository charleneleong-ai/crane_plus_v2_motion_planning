#!/usr/bin/env python
###
# File Created: Wednesday, February 6th 2019, 4:26:25 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By:
# Last Modified:
###


import sys
import rospy

def check_rosparams():
    """Checks for valid ROS parameters
    
    Args:
        mode (str): Mode of parameter tuning session
    """
    mode = rospy.get_param('~mode')
    modes = ['default', 'tpe', 'rand', 'ompl',
             'smac', 'aucbandit', 'gp', 'rf', 'et', 'gbrt']
    if mode not in modes:
        rospy.logerr('Invalid mode.')
        rospy.logerr('Please choose from %s\n', str(modes))
        sys.exit(1)

    planner_select = rospy.get_param('~planner_select')
    if planner_select not in ['Cano_etal', 'Burger_etal']:
        rospy.logerr('Invalid planner config select.')
        rospy.logerr('Please choose from %s\n',
                     str(['Cano_etal', 'Burger_etal']))
        sys.exit(1)

    planners = ['all', 'BKPIECE', 'RRTConnect', 'KPIECE', 'BiTRRT']
    planner = rospy.get_param('~planner')
    if planner not in planners:
        rospy.logerr('Invalid planner.')
        rospy.logerr('Please choose from %s\n', str(planners))
        sys.exit(1)

    max_runtime = rospy.get_param('~max_runtime')
    if max_runtime != 'None':
        try:
            max_runtime = int(max_runtime)
        except ValueError:
            rospy.logerr(
                "Not an valid int value. Please express max_runtime in secs.\n")
            sys.exit(1)

    start_pose = rospy.get_param('~start_pose')
    target_pose = rospy.get_param('~target_pose')
    named_states = rospy.get_param('~named_states')

    if (start_pose == "None") and (target_pose == "None"):
        return
    # Will also return error if only start or only target is None
    elif start_pose not in named_states:
        rospy.logerr('start_pose not in list of named_states')
        rospy.logerr('Please choose from %s\n', str(named_states))
        sys.exit(1)
    elif target_pose not in named_states:
        rospy.logerr('target_pose not in list of named_states')
        rospy.logerr('Please choose from %s\n', str(named_states))
        sys.exit(1)

