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


class Param_Tuning_Session():
    def __init__(self, robot, move_group, planner_id):
        self.robot = robot
        self.move_group = move_group
        self.planner_id = planner_id
        self.params = None
        
    
    def get_planner_params(self):
        #rospy.loginfo('Waiting for get_planner_params')
        rospy.wait_for_service('get_planner_params')
        get_planner_params = rospy.ServiceProxy(
            'get_planner_params', GetPlannerParams)
        try:
            req = get_planner_params(self.planner_id, "arm")
            print(req)
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)

        return req.params

    
    def update_planner_params(self):
        #rospy.loginfo('Waiting for set_planner_params')
        rospy.wait_for_service('set_planner_params')
        set_planner_params = rospy.ServiceProxy(
            'set_planner_params', SetPlannerParams)
        try:
            set_planner_params(self.planner_id, "arm", self.params, True)
            rospy.loginfo('Parameters updated')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)

        return get_planner_params(self.planner_id)   

    def move_arm(self, target):
        ##Setting goal
        self.move_group.set_named_target(target)
        plan = self.move_group.plan()

        ##Visualising trajectory
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        rospy.loginfo("Moving to %s pose", target)
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

        def reset_state:
            pass
            

def init_arm():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()
    arm = moveit_commander.MoveGroupCommander("arm")
    return robot, arm
 

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

    session = Param_Tuning_Session(robot, arm, planner)
    #session.

    session.move_arm(target)


if __name__ == "__main__":
    main()
