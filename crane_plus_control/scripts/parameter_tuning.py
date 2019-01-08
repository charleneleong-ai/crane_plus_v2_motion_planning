#!/usr/bin/env python

import sys
import time
import argparse
import copy
import rospy
from math import pi
import numpy as np
import matplotlib.pyplot as plt

import moveit_commander
from moveit_commander.conversions import pose_to_list
import moveit_msgs.msg
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
import geometry_msgs.msg
from std_msgs.msg import String

import csv
from hyperopt import hp, rand, tpe, Trials, fmin
from timeit import default_timer as timer
from hyperopt.pyll.stochastic import sample


planners = ['RRTConnectkConfigDefault', 'BiTRRTkConfigDefault',
            'BKPIECEkConfigDefault', 'KPIECEkConfigDefault']

#temp
planner = planners[1]

class ParamTuningSession(object):

    def __init__(self, robot, move_group, planner_id):
        self.robot = robot
        self.move_group = move_group
        self.planner_id = planner_id
        self.params = self.get_planner_params()

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

    def set_planner_params(self):
        #rospy.loginfo('Waiting for set_planner_params')
        rospy.wait_for_service('set_planner_params')
        set_planner_params = rospy.ServiceProxy(
            'set_planner_params', SetPlannerParams)
        try:
            set_planner_params(self.planner_id, "arm", self.params, True)
            rospy.loginfo('Parameters updated')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)

        return self.get_planner_params()

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

    def reset_state(self):
        pass


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

def obj(x):
    """Objective function to minimize"""
    
    # Create the polynomial object
    f = np.poly1d([1, -2, -28, 28, 12, -26, 100])

    # Return the value of the polynomial
    return f(x) * 0.05

def optimise_obj_simple():
    # Space over which to evluate the function is -5 to 6
    x = np.linspace(-5, 6, 10000)
    y = obj(x)

    miny = min(y)
    minx = x[np.argmin(y)]

    # Print out the minimum of the function and value
    print('Minimum of %0.4f occurs at %0.4f' % (miny, minx))

    # Create the domain space
    space = hp.uniform('x', -5, 6)

    samples = []

    # Sample 10000 values from the range
    for _ in range(10000):
        samples.append(sample(space))

    # Histogram of the values
    plt.hist(samples, bins = 20, edgecolor = 'black'); 
    plt.xlabel('x'); plt.ylabel('Frequency'); plt.title('Domain Space');

    tpe_trials = Trials()
    rand_trials = Trials()

    start_time = timer()
        # Run 2000 evals with the tpe algorithm
    tpe_best = fmin(fn=obj, space=space, algo=tpe.suggest, trials=tpe_trials, 
                    max_evals=2000, rstate= np.random.RandomState(50))

    tpe_run_time = timer() - start_time
    start_time = timer()
                    # Run 2000 evals with the random algorithm
    rand_best = fmin(fn=obj, space=space, algo=rand.suggest, trials=rand_trials, 
                 max_evals=2000, rstate= np.random.RandomState(50))

    rand_run_time = timer() - start_time

    print(tpe_run_time)
    print(rand_run_time)

    # Print out information about losses
    print('Minimum loss attained with TPE:    {:.4f}'.format(tpe_trials.best_trial['result']['loss']))
    print('Minimum loss attained with random: {:.4f}'.format(rand_trials.best_trial['result']['loss']))
    print('Actual minimum of f(x):            {:.4f}'.format(miny))

    # Print out information about number of trials
    print('\nNumber of trials needed to attain minimum with TPE:    {}'.format(tpe_trials.best_trial['misc']['idxs']['x'][0]))
    print('Number of trials needed to attain minimum with random: {}'.format(rand_trials.best_trial['misc']['idxs']['x'][0]))

    # Print out information about value of x
    print('\nBest value of x from TPE:    {:.4f}'.format(tpe_best['x']))
    print('Best value of x from random: {:.4f}'.format(rand_best['x']))
    print('Actual best value of x:      {:.4f}'.format(minx))



def main():
    # robot, arm = init_arm()
    # #target = check_target()
    # poses = rospy.get_param("/parameter_tuning/poses")
    
    # optimise_obj()
    #session = ParamTuningSession(robot, arm, planner)
    #session.

    #session.move_arm(target)


if __name__ == "__main__":
    main()
