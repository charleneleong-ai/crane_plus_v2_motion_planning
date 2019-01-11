#!/usr/bin/env python

import sys
from timeit import default_timer as timer
from collections import OrderedDict
import itertools

import csv
import json
import pprint
import pandas as pd
import numpy as np
from functools import partial

import rospkg
import rospy
import std_msgs.msg

import moveit_commander
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
import moveit_msgs.msg

from hyperopt import hp, rand, tpe, Trials, fmin, STATUS_OK
from hyperopt.pyll.stochastic import sample

# from objectives import objectives


class ParamTuningSession(object):
    """
    Class for a Parameter Tuning Session
    """

    def __init__(self):
        self.planner_config_obj = PlannerConfig()
        self.mode = self.planner_config_obj.mode
        self.name = self.planner_config_obj.name
        self.planner_config = self.planner_config_obj.planner_config
        self.planners = self.planner_config_obj.planners

        self.n_trial = 0
        if self.mode != "baseline":
            self.max_trials = rospy.get_param("~max_trials")
        self.iter = rospy.get_param("~iter")
        self.start_pose = self.planner_config_obj.start_pose
        self.target_pose = self.planner_config_obj.target_pose

        self.results_path = rospkg.RosPack().get_path(
            'crane_plus_control')+'/results/'+self.name+".csv"
        self.results_df = {}

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")
        # self.planning_frame = self.group.get_planning_frame()  # "/world"
        # self.scene = moveit_commander.PlanningSceneInterface()

    def _move_arm(self, pose):
        self.group.set_named_target(pose)
        plan = self.group.plan()
        display_trajectory_publisher = rospy.Publisher('/group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

        #rospy.loginfo("Moving to %s pose", pose)
        self.group.go(wait=True)
        self.group.stop()

    def _plan_path(self, start_pose, target_pose):
        # start_state = self.robot.get_current_state()
        # start_state.joint_state.position = start_pose
        # self.group.set_start_state(start_state)
        self.group.set_named_target(target_pose)
        start_time = timer()
        planned_path = self.group.plan()
        plan_time = timer() - start_time

        # not sure how to figure out a failure otherwise
        if len(planned_path.joint_trajectory.points) != 0:
            length = self._get_path_length(planned_path)
            # success = 1

        return {"path": planned_path, "plan_time": plan_time, "length": length}

    def _get_path_length(self, path):
        """
        Function to calculate both dist path lengths and actual path both in jointspace and workspace
        :param path: plan from move_group
        :return: dict of lengths in jointspace and workspace
        """
        pts = path.joint_trajectory.points
        j_length = 0    # j = jointspace
        w_length = 0    # w = workspace
        n_pts = len(pts)
        a = np.array(pts[0].positions)      # Straight path
        b = np.array(pts[n_pts-1].positions)
        j_dist = np.linalg.norm((a - b), ord=2)         # Get 2-norm (Euc dist)
        # a1 = np.array(self._get_forward_kinematics(a))    # Get 3D fwd kinematics coordinates
        # b1 = np.array(self._get_forward_kinematics(b))
        # w_dist = np.linalg.norm((a1 - b1), ord=2)

        for x in xrange(n_pts-1):       # Actual path
            a = np.array(pts[x].positions)
            b = np.array(pts[x+1].positions)
            j_length += np.linalg.norm((a - b), ord=2)
            # a1 = np.array(self._get_forward_kinematics(a))
            # b1 = np.array(self._get_forward_kinematics(b))
            # w_length += np.linalg.norm((a1 - b1), ord=2)
        # return {'joint_dist': j_dist, 'joint_path': j_length,
        #         'work_dist': w_dist, 'work_path': w_length}
        return {'joint_dist': j_dist, 'joint_length': j_length}

    # def _get_forward_kinematics(self, joint_pos):
    #     """
    #     Function that gets the forward kinematics using the move_group service
    #     """
    #     rospy.wait_for_service('compute_fk')
    #     try:
    #         moveit_fk = rospy.ServiceProxy('compute_fk', moveit_msgs.srv._GetPositionFK.GetPositionFK)
    #     except rospy.ServiceException, e:
    #         rospy.logerror("Service call failed: %s"%e)

    #     fkln = ['ee_link']   #forward kinematic link to be calculated

    #     header = std_msgs.msg.Header(0,rospy.Time.now(),"/world")   #make header for the argument to moveit_fk

    #     rs = self.robot.get_current_state()
    #     rs.joint_state.position = joint_pos   #robot state for argument to moveit_fk

    #     fwd_kin = moveit_fk(header, fkln,rs)

    #     print(fwd_kin)
    #     try:
    #         pos = fwd_kin.pose_stamped[0].pose.position   #extract position
    #         fwd_kin_coordinates = [pos.x, pos.y, pos.z]
    #     except IndexError:
    #         fwd_kin_coordinates = [0, 0, 0]

    #     return fwd_kin_coordinates

    def _get_stats(self, start_pose, target_pose):

        run_times = []
        path_stats = []
        for _ in xrange(self.iter):
            self._move_arm(start_pose)  # reset to start pose
            start_time = timer()
            path = self._plan_path(start_pose, target_pose)
            self._move_arm(target_pose)
            run_time = timer() - start_time

            run_times.append(run_time)
            path_stats.append(path)

        #pprint.pprint(path_stats)
        avg_run_time = sum(run_times)/float(len(run_times))
        avg_plan_time = float(sum(d['plan_time']
                                  for d in path_stats)) / len(path_stats)
        avg_dist = float(
            sum(d['length']['joint_dist'] for d in path_stats)) / len(path_stats)
        avg_path_length = float(
            sum(d['length']['joint_length'] for d in path_stats)) / len(path_stats)
        result = {'avg_run_time': avg_run_time, 'avg_plan_time': avg_plan_time,
                  'avg_dist': avg_dist, 'avg_path_length': avg_path_length}

        return result

    def _objective(self, params):
        # raise NotImplementedError, "Should be implemented in child class"
        self.n_trial += 1
        # Extract param set
        params_config = params['params_config']
        params_set = params['params_set']

        # Set new params
        planner_params = moveit_msgs.msg.PlannerParams()
        planner_params.keys = params_set.keys()
        planner_params.values = [str(i) for i in params_set.values()]
        self.group.set_planner_id(params_config['planner'])
        self.planner_config_obj.set_planner_params(
            params_config['planner'], planner_params)
        # print(self._get_planner_params(params['planner']))

        # Execute experiment for iter times and get planning and run_time stats
        stats = self._get_stats(
            params_config['start_pose'], params_config['target_pose'])

        loss = sum(stats.values())

        rospy.loginfo("n_trial: %d loss: %.4f avg_run_time: %.4f avg_plan_time: %.4f avg_dist: %.4f avg_path_length: %.4f",
                      self.n_trial, loss, stats['avg_run_time'], stats['avg_plan_time'], stats['avg_dist'], stats['avg_path_length'])

        # Create OrderedDict to write to CSV
        result = OrderedDict([('n_trial', self.n_trial), ('loss', loss)])
        planner_params = OrderedDict([('planner', params_config['planner']), ('start_pose', params_config['start_pose']), (
            'target_pose', params_config['target_pose']), ('avg_runs', self.iter)])
        stats = OrderedDict([('avg_runtime', stats['avg_run_time']), ('avg_plan_time', stats['avg_plan_time']),
                             ('avg_dist', stats['avg_dist']), ('avg_path_length', stats['avg_path_length'])])
        result = OrderedDict(list(result.items()) +
                             list(planner_params.items() + list(stats.items())))
        params_set_str = str(params_set)

        result_csv = OrderedDict(list(result.items()) + [('params', params_set_str)])
        result = OrderedDict(list(result.items()) + [('params', params_set), ('status', STATUS_OK)])

        print(json.dumps(result_csv, indent=4))
        result_df = pd.DataFrame(dict(result_csv), columns=result.keys(), index=[0])

        with open(self.results_path, 'a') as f:
            result_df.to_csv(f, header=False, index=False)

        return dict(result)

    def _optimise_obj(self, start_pose, target_pose):
        # Write headers
        with open(self.results_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['n_trial', 'loss', 'planner', 'start_pose', 'target_pose', 'avg_runs', 'avg_runtime',
            'avg_plan_time', 'avg_dist', 'avg_path_length', 'params'])

        # Setting up the parameter search space and parameters
        for planner, params_set in self.planner_config.iteritems():
            params_set = dict(self.planner_config[planner].items())
            for k, v in params_set.iteritems():
                if isinstance(v, list):
                    begin_range = v[0]
                    end_range = v[1]
                    step = v[2]
                    # Discrete uniform dist
                    params_set[k] = hp.quniform(
                        k, begin_range, end_range, step)
            params = {}
            params['params_set'] = params_set
            params['params_config'] = {
                'planner': planner, 'start_pose': start_pose, 'target_pose': target_pose, 'avg_runs': self.iter}

            print("\n")
            rospy.loginfo("Executing %s on %s:  Max trials: %d Averaging over %d runs", self.mode,
                          params['params_config']['planner'], self.max_trials, params['params_config']['avg_runs'])
            self.n_trial = 0        # Reset to n_trials to zero for each planner
            if self.mode == "tpe":
                algo = partial(tpe.suggest,
                               # Sample 1000 candidate and select candidate that has highest Expected Improvement (EI)
                               n_EI_candidates=100,
                               # Use 20% of best observations to estimate next set of parameters
                               gamma=0.2,
                               # First 20 trials are going to be random
                               n_startup_jobs=20)
            elif self.mode == "rand":
                algo = rand.suggest

            trials = Trials()
            best = fmin(fn=self._objective, space=params, algo=algo,
                        max_evals=self.max_trials, trials=trials)

        #     self.results_df[planner] = trials.results
        #     # pprint.pprint(trials.results)

        # pprint.pprint(self.results_df)
        # #self.results_df.to_csv(self.results_path, index=False)

    def _obtain_baseline(self, start_pose, target_pose):
        # for OMPL defaults
        # self.planner_config = rospy.get_param('/group/planner_configs/')
        # self.planner_config = dict((k, self.planner_config[k]) for k in self.planners if k in self.planner_config)

        #avg_run_times = []
        for p in self.planners:
            rospy.loginfo(
                "Executing %s baseline: Averaging over %d runs", p, self.iter)
            self.group.set_planner_id(p)

            # avg_run_time = self._get_avg_run_time(
            #     start_pose, target_pose, iter)
            # avg_run_times.append(avg_run_time)

            self.planner_config[p].pop('type', None)

        result = OrderedDict([('avg_run_time', avg_run_time), ('planner', p), ('start_pose', start_pose),
                              ('target_pose', target_pose), ('avg_runs', self.iter), ('params', self.planner_config.values())])

        self.results_df = pd.DataFrame(result)
        self.results_df.to_csv(self.results_path, index=False)

        avg_run_times = pd.DataFrame(
            {'planners': self.planners, 'run_times (s)': avg_run_times})
        print("\n")
        print(avg_run_times)

        return df, avg_run_times

    def run(self):
        if self.mode == "baseline":
            self._obtain_baseline(self.start_pose, self.target_pose)
        else:
            self._optimise_obj(self.start_pose, self.target_pose)

    def get_results(self):
        # try:
        # catch:

        results = pd.read_csv(self.results_path)
        # Sort with best scores on top and reset index for slicing

        for p in results.planner.unique():
            planner_df = results[results['planner'] == p]
            #print(planner_df)
            planner_df.sort_values(
                'avg_run_time', ascending=True, inplace=True)
            planner_df.reset_index(inplace=True, drop=True)
            print(planner_df.head())


class PlannerConfig(object):
    def __init__(self):
        self.planning_time = 2  # seconds

        self.planner_select = rospy.get_param(
            "~planner_config")
        if self.planner_select not in ['Cano_etal']:
                rospy.logerr("Invalid planner config select")
                sys.exit(1)

        self.start_pose = rospy.get_param("~start_pose")
        self.target_pose = rospy.get_param("~target_pose")
        self.named_states = rospy.get_param("~named_states")
        if self.target_pose not in self.named_states:
            rospy.logerr('target_pose not in list of named_states')
            rospy.logerr(self.named_states)
            sys.exit(1)
        elif self.start_pose not in self.named_states:
            rospy.logerr('start_pose not in list of named_states')
            rospy.logerr(self.named_states)
            sys.exit(1)

        self.mode = rospy.get_param("~mode")
        if self.mode not in ['baseline', 'tpe', 'rand']:
            rospy.logerr("Invalid mode.")
            sys.exit(1)

        if self.mode is "baseline":
            self.planner_config = rospy.get_param(
                "~planner_configs_"+self.planner_select+"_default")
        else:
            self.planner_config = rospy.get_param(
                "~planner_configs_"+self.planner_select+"_tune")
        assert isinstance(self.planner_config, dict)

        self.planners = self.planner_config.keys()
        self.name = self.planner_select+"_"+self.mode

    def get_planner_params(self, planner_id):
        # rospy.loginfo('Waiting for get_planner_params')
        rospy.wait_for_service('get_planner_params')
        get_planner_params = rospy.ServiceProxy(
            'get_planner_params', GetPlannerParams)
        try:
            req = get_planner_params(planner_id, "arm")
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
