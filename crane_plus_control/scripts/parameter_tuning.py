#!/usr/bin/env python

import sys
import os
import time
import argparse
import copy
import rospy
import rospkg
from math import pi
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import pprint

import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
import geometry_msgs.msg
from std_msgs.msg import String, Header

import csv
import itertools
from hyperopt import hp, rand, tpe, Trials, fmin
from timeit import default_timer as timer
from hyperopt.pyll.stochastic import sample


planners = ['RRTConnectkConfigDefault', 'BiTRRTkConfigDefault',
            'BKPIECEkConfigDefault', 'KPIECEkConfigDefault']

#temp
planner = planners[1]


class ParamTuningSession(object):

    def __init__(self, robot, move_group, planners):
        self.robot = robot
        self.move_group = move_group
        self.planners = planners
        

    def get_planner_params(self, planner_id):
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

    def set_planner_params(self, planner_id, params):
        #rospy.loginfo('Waiting for set_planner_params')
        rospy.wait_for_service('set_planner_params')
        set_planner_params = rospy.ServiceProxy(
            'set_planner_params', SetPlannerParams)
        try:
            set_planner_params(planner_id, "arm", params, True)
            rospy.loginfo('Parameters updated')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)
        print(params)

    def move_arm(self, pose):
        ##Setting goal
        # self.set_state()
        
        self.move_group.set_named_target(pose)
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

        rospy.loginfo("Moving to %s pose", pose)
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()


    def obtain_baseline(self, start, target):
        planner_configs_default = {}
        run_times = []

        for p in self.planners:

            planner_configs_default[p] = rospy.get_param(
                "/move_group/planner_configs/"+p)
            planner_configs_default[p].pop('type', None)
            
            self.move_arm(start)
            start_time = timer()
            self.move_arm(target)
            run_time = timer() - start_time
            run_times.append(run_time)

            planner_configs_default[p]['run_time (s)'] = run_time
            planner_configs_default[p]['start'] = start
            planner_configs_default[p]['target'] = target

        #pprint.pprint(planner_configs_default)

        #converting nested dict to pd DataFrame
        ids = []
        frames = []
        for id, d in sorted(planner_configs_default.iteritems(), key=lambda (k,v): (v,k)):
            ids.append(id)
            frames.append(pd.DataFrame.from_dict(d, orient='index'))

        df = pd.concat(frames, keys=ids)
        run_times = pd.DataFrame({'planners' : planners, 'run_times (s)' : run_times})
        print(run_times)

        #saving to csv in benchmarks folder
        pkg_path = rospkg.RosPack().get_path('crane_plus_control')
        df.to_csv(pkg_path+"/benchmarks/ompl_baseline.csv", sep='\t')
        
        return df, run_times 


# class PlannerConfigs(Object):
#     def __init__(self, name, planners):
#         self.name = name
#         self.planners = planners

def init_arm():
    """
    Intialises the rospy and RobotCommander and Movegroup Commander
    :return robot: instance of RobotCommander
    :return arm: instance of MoveGroupCommander
    """
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning',
                    anonymous=True)
    robot = moveit_commander.RobotCommander()
    #scene = moveit_commander.PlanningSceneInterface()
    arm = moveit_commander.MoveGroupCommander("arm")
    return robot, arm


def check_target():
    """
    Validates if selected parameter is in list of named states from rosparam server, else exits program
    :return target: target pose of robot
    """
    target = rospy.get_param("/parameter_tuning/target")
    named_states = rospy.get_param("/parameter_tuning/named_states")

    if target not in named_states:
        rospy.logerr('Target not in list of named_states')
        rospy.logerr(named_states)
        sys.exit(1)

    return target


def objective(params, poses):

    global ITERATION
    ITERATION += 1

    start_time = timer()

    session = ParamTuningSession(robot, arm, planner)
    session.set_planner_params(params)
    session.move_arm('vertical')

    for pose in poses:
        session.move_arm(pose)

    run_time = timer() - start_time

    #min run_time
    loss = 1 - run_time

    return {'loss': loss, 'params': params, 'iteration': ITERATION, 'run_time': run_time, 'status': STATUS_OK}



def main():
    robot, arm = init_arm()
    target = check_target()

    #check if move to defined target or load all params
    if target is "None":
        pass

    # poses = rospy.get_param("/parameter_tuning/poses")

    #planners, params = load_params()
    planner_configs_param_tune = rospy.get_param(
        "/parameter_tuning/planner_configs_param_tune")
    planners = planner_configs_param_tune.keys()
    session = ParamTuningSession(robot, arm, planners)
    baseline = session.obtain_baseline('vertical', target)
    
    # for p in planners:
    #     print(p)
    #     planner_configs[p].pop('type', None)
    #     params = dict(planner_configs[p].items())
    #print(params, type(params))
    # for key, val in params:
    #     # if not isinstance(val, (str)):
    #     print(key, val, type(val))

    # optimise_obj()


if __name__ == "__main__":
    main()
