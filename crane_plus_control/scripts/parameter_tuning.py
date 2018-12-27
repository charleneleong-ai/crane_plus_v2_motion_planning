#!/usr/bin/env python

import sys
import time
import argparse
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

planners = ['RRTConnectkConfigDefault', 'BiTRRTkConfigDefault',
            'BKPIECEkConfigDefault', 'KPIECEkConfigDefault']

#temp
planner = planners[1]


def init_arm():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning',
                    anonymous=True)
    planner = planners[1]
    target = 'resting'
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    arm = moveit_commander.MoveGroupCommander("arm")
    return robot, arm


def update_planner_params(planner_id, params):
    #rospy.loginfo('Waiting for set_planner_params')
    rospy.wait_for_service('set_planner_params')
    set_planner_params = rospy.ServiceProxy(
        'set_planner_params', SetPlannerParams)
    try:
        req = set_planner_params(planner_id, "arm", params, True)
        rospy.loginfo('Parameters updated')
    except rospy.ServiceException as e:
        rospy.logerr('Failed to get params: %s', e)

    return get_planner_params(planner_id)


def get_planner_params(planner_id):
    #rospy.loginfo('Waiting for get_planner_params')
    rospy.wait_for_service('get_planner_params')
    get_planner_params = rospy.ServiceProxy(
        'get_planner_params', GetPlannerParams)
    try:
        req = get_planner_params(planner_id, "arm")
        print(req)
    except rospy.ServiceException as e:
        rospy.logerr('Failed to get params: %s', e)

    return req.params


def move_arm(robot, arm, target):
    ##Setting goal
    arm.set_named_target(target)
    plan = arm.plan()

    ##Visualising trajectory
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    rospy.loginfo("Moving to %s pose", target)
    plan = arm.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    arm.stop()


def check_target():
    target = rospy.get_param("/parameter_tuning/target")
    named_states = rospy.get_param("/parameter_tuning/named_states")
    
    if target not in named_states:
        rospy.logerr('Target not in list of named_states')
        rospy.logerr(named_states)
        sys.exit(1)

    return target

def main():
    robot, arm = init_arm()
    target = check_target()
    # params = get_planner_params(planner)
    # new_params = params
    # new_params.values = ['1e300', '0.1', '0', '100', '0', '0.1', 'geometric::BiTRRT']
    # update_planner_params(planner, new_params)
    #move_arm(robot, arm, target)


if __name__ == "__main__":
    ##for rosrun
    # parser = argparse.ArgumentParser()
    # parser.add_argument('--target', help='Enter valid target pose', required=True)
    # args = parser.parse_args()
    # main(args.target)
    main()
