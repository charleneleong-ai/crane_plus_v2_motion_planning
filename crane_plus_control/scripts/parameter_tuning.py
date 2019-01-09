#!/usr/bin/env python

import sys
from timeit import default_timer as timer
import csv
import pprint
import random
import pandas as pd
import numpy as np

import rospkg
import rospy

import moveit_commander
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
import moveit_msgs.msg

from hyperopt import hp, rand, tpe, Trials, fmin, STATUS_OK
from hyperopt.pyll.stochastic import sample

PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')
MAX_EVALS = 5
EVAL = 0


class ParamTuningSession(object):

    def __init__(self, robot, move_group, planner_config, tune):
        self.robot = robot
        self.move_group = move_group
        self.tune = tune

        if self.tune == True:
            self.name = planner_config+"_tune"
            self.planner_config = rospy.get_param(
                "/parameter_tuning/planner_configs_"+planner_config+"_tune")
            assert isinstance(self.planner_config, dict)
        else:
            self.name = planner_config+"_default"
            self.planner_config = rospy.get_param(
                "/parameter_tuning/planner_configs_"+planner_config+"_default")
            assert isinstance(self.planner_config, dict)
        self.planners = self.planner_config.keys()
        self.results_path = PKG_PATH+'/results/'+self.name+'_tpe.csv'

        for p in self.planners:
            self.planner_config[p].pop('type', None)
            #pprint.pprint(self.planner_config[p])

    def get_planner_params(self, planner_id):
        # rospy.loginfo('Waiting for get_planner_params')
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
        # rospy.loginfo('Waiting for set_planner_params')
        rospy.wait_for_service('set_planner_params')
        set_planner_params = rospy.ServiceProxy(
            'set_planner_params', SetPlannerParams)
        try:
            set_planner_params(planner_id, "arm", params, True)
            rospy.loginfo('Parameters updated')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)
        print(params)

    def _move_arm(self, pose):
        # Setting goal
        self.move_group.set_named_target(pose)
        plan = self.move_group.plan()

        # Visualising trajectory
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

        #rospy.loginfo("Moving to %s pose", pose)
        plan = self.move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()

    def _get_avg_run_time(self, start_pose, target_pose, iter):
        run_times = []
        for _ in range(iter):
            self._move_arm(start_pose)
            start_time = timer()
            self._move_arm(target_pose)
            run_time = timer() - start_time
            run_times.append(run_time)

        return sum(run_times)/float(len(run_times))

    def _obtain_baseline(self, start_pose, target_pose, iter):
        avg_run_times = []
        for p in self.planners:
            #rospy.loginfo("Executing %s: Averaging over %d runs", p, iter)

            avg_run_time = self._get_avg_run_time(
                start_pose, target_pose, iter)
            avg_run_times.append(avg_run_time)
            # Add info to dict
            self.planner_config[p]['runtime (s) avg_' +
                                   str(iter)] = avg_run_time
            self.planner_config[p]['start_pose'] = start_pose
            self.planner_config[p]['target_pose'] = target_pose

        # Converting nested dict to pd DataFrame
        ids = []
        frames = []
        for id, d in sorted(self.planner_config.iteritems(), key=lambda (k, v): (v, k)):
            ids.append(id)
            frames.append(pd.DataFrame.from_dict(d, orient='index'))

        df = pd.concat(frames, keys=ids)

        avg_run_times = pd.DataFrame(
            {'planners': self.planners, 'run_times (s)': avg_run_times})
        print("\n")
        print(avg_run_times)

        df.to_csv(PKG_PATH+"/results/"+self.name+"_baseline.csv", sep='\t')

        return df, avg_run_times

    def _objective(self, params):
        # Keep track of evals
        global EVAL
        EVAL += 1

        avg_run_time = self._get_avg_run_time(
            params['start_pose'], params['target_pose'], params['avg_runs'])

        rospy.loginfo("EVAL: %d Avg Runtime: %f",
                      EVAL, avg_run_time)

        result = {'EVAL': EVAL, 'loss': avg_run_time, 'planner': params['planner'], 'start_pose': params['start_pose'],
                  'target_pose': params['target_pose'], 'avg_runs': params['avg_runs']}

        params_set = params.copy()
        params_set.pop('planner', None)
        params_set.pop('start_pose', None)
        params_set.pop('target_pose', None)
        params_set.pop('avg_run_time', None)
        params_set.pop('avg_runs', None)
        result['params'] = params_set
        result['status'] = STATUS_OK

        of_connection = open(self.results_path, 'a')
        writer = csv.writer(of_connection)
        writer.writerow([EVAL, avg_run_time, params['planner'], params['start_pose'],
                         params['target_pose'], params['avg_runs'], params_set])

        return result

    def _optimise_obj(self, start_pose, target_pose, iter):
        # File to save first results
        of_connection = open(self.results_path, 'w')
        writer = csv.writer(of_connection)
        writer.writerow(['EVAL', 'avg_run_time', 'planner',
                         'start_pose', 'target_pose', 'avg_runs', 'params'])
        of_connection.close()

        # Setting up the parameter search space and parameters
        for planner, params in self.planner_config.iteritems():
            params = dict(self.planner_config[planner].items())
            # pprint.pprint(param_grid)
            #print(planner)

            for k, v in params.iteritems():
                if isinstance(v, list):
                    begin_range = v[0]
                    end_range = v[1]
                    step = v[2]
                    # Discrete uniform dist
                    params[k] = hp.quniform(k, begin_range, end_range, step)

            params['planner'] = planner
            params['start_pose'] = start_pose
            params['target_pose'] = target_pose
            params['avg_runs'] = iter
            # pprint.pprint(params)

            print("\n")
            rospy.loginfo("Executing %s:  Max Eval: %d Averaging over %d runs",
                          params['planner'], MAX_EVALS, params['avg_runs'])
            # Reset to num_evals to zero for each planner
            global EVAL
            EVAL = 0
            #Run optimisation
            tpe_trials = Trials()
            best = fmin(fn=self._objective, space=params, algo=tpe.suggest,
                        max_evals=MAX_EVALS, trials=tpe_trials)
            # pprint.pprint(best)

    def run(self, start_pose, target_pose, iter=3):
        if self.tune == True:
            self._optimise_obj(start_pose, target_pose, iter)
        else:
            self._obtain_baseline(start_pose, target_pose, iter)

    def get_results(self):
        results = pd.read_csv(self.results_path)
        # Sort with best scores on top and reset index for slicing
        results.sort_values('loss', ascending=True, inplace=True)
        results.reset_index(inplace=True, drop=True)
        print(results.head())


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
    :return target_pose: target_pose of robot
    """
    start_pose = rospy.get_param("/parameter_tuning/start_pose")
    target_pose = rospy.get_param("/parameter_tuning/target_pose")
    named_states = rospy.get_param("/parameter_tuning/named_states")

    if target_pose not in named_states:
        rospy.logerr('target_pose not in list of named_states')
        rospy.logerr(named_states)
        sys.exit(1)

    elif start_pose not in named_states:
        rospy.logerr('start_pose not in list of named_states')
        rospy.logerr(named_states)
        sys.exit(1)

    return start_pose, target_pose


def init_planner():
    planner_config = rospy.get_param("/parameter_tuning/planner_config")
    tune = rospy.get_param("/parameter_tuning/tune")
    return planner_config, tune


def main():
    robot, arm = init_arm()
    start_pose, target_pose = check_valid_pose()
    planner_config, tune = init_planner()

    #check if move to defined target_pose or load all params
    if target_pose is "None":
        pass

    # poses = rospy.get_param("/parameter_tuning/poses")

    session = ParamTuningSession(robot, arm, planner_config, tune=tune)
    session.run(start_pose, target_pose, iter=1)
    # session.get_results()


if __name__ == "__main__":
    main()
