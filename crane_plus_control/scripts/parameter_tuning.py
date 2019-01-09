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


class ParamTuningSession(object):

    def __init__(self, robot, move_group, planner_config):
        self.robot = robot
        self.move_group = move_group
        self.name = planner_config
        self.planner_config = rospy.get_param("/parameter_tuning/"+planner_config)
        self.planners = self.planner_config.keys()
        
        for p in self.planners:
            self.planner_config[p].pop('type', None)
            #pprint.pprint(self.planner_config[p])

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


    def obtain_baseline(self, start, target, i):

        avg_run_times = []
        for p in self.planners:
            rospy.loginfo("Executing %s: Averaging over %d runs", p, i)

            run_times = []
            for _ in range(i):
                self.move_arm(start)
                start_time = timer()
                self.move_arm(target)
                run_time = timer() - start_time
                run_times.append(run_time)

            avg_run_time = sum(run_times)/float(len(run_times))
            avg_run_times.append(avg_run_time) 

            self.planner_config[p]['baseline_runtime (s) avg_'+str(i)] = avg_run_time
            self.planner_config[p]['start'] = start
            self.planner_config[p]['target'] = target

        #converting nested dict to pd DataFrame
        ids = []
        frames = []
        for id, d in sorted(self.planner_config.iteritems(), key=lambda (k,v): (v,k)):
            ids.append(id)
            frames.append(pd.DataFrame.from_dict(d, orient='index'))

        df = pd.concat(frames, keys=ids)
        
        avg_run_times = pd.DataFrame({'planners' : self.planners, 'run_times (s)' : avg_run_times})
        print("\n")
        print(avg_run_times)

        #saving to csv in benchmarks folder
        pkg_path = rospkg.RosPack().get_path('crane_plus_control')
        df.to_csv(pkg_path+"/benchmarks/"+self.name+"_baseline.csv", sep='\t')
        
        return df, avg_run_times 


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


def check_valid_pose():
    """
    Validates if selected parameter is in list of named states from rosparam server, else exits program
    :return target: target pose of robot
    """
    start = rospy.get_param("/parameter_tuning/start")
    target = rospy.get_param("/parameter_tuning/target")
    named_states = rospy.get_param("/parameter_tuning/named_states")

    if target not in named_states:
        rospy.logerr('Target not in list of named_states')
        rospy.logerr(named_states)
        
    elif start not in named_states:
        rospy.logerr('Start not in list of named_states')
        rospy.logerr(named_states)
        
    return start, target


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
    start, target = check_valid_pose()

    #check if move to defined target or load all params
    if target is "None":
        pass

    # poses = rospy.get_param("/parameter_tuning/poses")

    #planners, params = load_params()
    # planner_configs_param_tune = rospy.get_param(
    #     "/parameter_tuning/planner_configs_param_tune")
    # planners = planner_configs_param_tune.keys()

    planner_config = 'planner_configs_Cano_etal_default'

    session = ParamTuningSession(robot, arm, planner_config)
    session.obtain_baseline(start, target, 5)

    # for p in planners:
    #     print(p)
    #     planner_configs[p].pop('type', None)
    #     params = dict(planner_configs[p].items())
    # print(params, type(params))
    # for key, val in params:
    #     # if not isinstance(val, (str)):
    #     print(key, val, type(val))

    # optimise_obj()


if __name__ == "__main__":
    main()
