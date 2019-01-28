#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 7:18:59 pm
# Author: Charlene Leong
# Last Modified: Monday, January 28th 2019, 3:44:34 pm
# Modified By: Charlene Leong
###

import sys
import time
import os
import csv
import pprint
from timeit import default_timer as timer
from collections import OrderedDict
import pandas as pd
import numpy as np

import rospkg
import rospy

import moveit_commander
import moveit_msgs.msg

from hyperopt import STATUS_OK

from planner_config import PlannerConfig
from scene_object import Scene

ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')


class Session(object):
    """
    Session Base Class
    """

    def __init__(self):
        self.planner_config_obj = PlannerConfig()
        self.planner_config = self.planner_config_obj.planner_config
        self.planner_select = self.planner_config_obj.planner_select
        self.planners = self.planner_config_obj.planners

        self.mode = rospy.get_param('~mode')
        self.avg_runs = rospy.get_param('~avg_runs')
        self.max_runtime = rospy.get_param('~max_runtime')
        self.n_trial = 0
        self.max_plan_time = rospy.get_param('~max_plan_time')
        

        if self.mode not in ['default', 'ompl']:
            self.max_trials = rospy.get_param('~max_trials')
            if self.max_runtime != 'None':
                self.max_runtime = int(self.max_runtime)
                self.max_trials = 10000     # Set to arbitrary large number

        self.start_pose = rospy.get_param('~start_pose')
        self.target_pose = rospy.get_param('~target_pose')
        self.path_tune = False
        if (self.start_pose != "None") and (self.target_pose != "None"):
            self.path_tune = True

        self.scenes = ['narrow_passage']

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.group.set_planning_time(self.max_plan_time)
        self.planning_frame = self.group.get_planning_frame()
        self.display_trajectory_publisher = rospy.Publisher('/group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        
        raw_dir = ROS_PKG_PATH + '/results/raw'
        if not os.path.exists(raw_dir):
            os.makedirs(raw_dir)

        self.results_path = raw_dir+'/'+self.planner_select+'_'+self.mode+'.csv'

    def run(self):
        raise NotImplementedError

    def _objective(self, params):
        self.n_trial += 1
        planner = params['planner']
        params_set = params['params_set']
        self.planner_config_obj.set_planner_params(planner, params_set)

        if self.path_tune:
            results = self._get_stats(self.start_pose, self.target_pose)
            stats = {'t_avg_run_time': results['avg_run_time'], 't_avg_plan_time': results['avg_plan_time'],
                     't_avg_dist': results['avg_dist'], 't_avg_path_length': results['avg_path_length'],
                     't_avg_success': results['avg_success']}
        else:
            results, stats = self.run_problem_set(planner_id=planner)

        loss = self._loss(planner, stats)

        current = time.time()
        elapsed_time = (current - params['start_time'])

        rospy.loginfo('elapsed_time(s): %.4f n_trial: %d loss: %.4f t_avg_run_time: %.4f t_avg_plan_time: %.4f t_avg_dist: %.4f t_avg_path_length: %.4f t_avg_success: %.4f\n',
                      elapsed_time, self.n_trial, loss, stats['t_avg_run_time'], stats['t_avg_plan_time'], stats['t_avg_dist'], stats['t_avg_path_length'], stats['t_avg_success'])

        # Create OrderedDict to write to CSV
        result = OrderedDict([('elapsed_time', elapsed_time), ('n_trial', self.n_trial), ('loss', loss), ('planner', planner), ('avg_runs', self.avg_runs),
                              ('t_avg_run_time', stats['t_avg_run_time']), (
                                  't_avg_plan_time', stats['t_avg_dist']),
                              ('t_avg_dist', stats['t_avg_dist']), (
                                  't_avg_path_length', stats['t_avg_path_length']),
                              ('t_avg_success', stats['t_avg_success'])])
        # Save params as str for csv export
        result_csv = OrderedDict(
            list(result.items()) + [('params', str(params_set))])
        # Saving results dict with STATUS_OK for hyperopt
        result = OrderedDict(list(result.items()) +
                             [('params', params_set), ('status', STATUS_OK)])
        # print(json.dumps(result_csv, indent=4))     # Print OrderedDict nicely

        result_df = pd.DataFrame(
            dict(result_csv), columns=result.keys(), index=[0])
        with open(self.results_path, 'a') as f:
            result_df.to_csv(f, header=False, index=False)

        if (self.max_runtime != "None") and (time.time() > params['end_time']):
            rospy.loginfo(
                'Program has run for allotted time (%d secs)', self.max_runtime)
            print('\n')
            rospy.loginfo('Saved results to %s', self.results_path)
            print('\n')
            sys.exit(0)

        return dict(result)

    def _loss(self, planner, stats):
        sucess_weight = 5
        success_rate = (1 - stats['t_avg_success']) * \
            sucess_weight        # Want to max this

        # If no success rate penalty is proportional to how badly the plan failed (how short the plan time is).
        if stats['t_avg_success'] == 0:
            loss = self.max_plan_time

        elif(stats['t_avg_success'] != 1):
            loss = loss*success_rate       # Penalise loss relative to success rate

        elif(stats['t_avg_success'] == 1):
            loss = stats['t_avg_plan_time']     # Plan time is our metric
        return loss
    
    def _write_headers(self, headers, results_path):
        with open(results_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(headers)


    def run_problem_set(self, planner_id, save=False, results_path=''):
        """Returns resuls after iterating over all scenes and corresponding states
        
        Args:
            planner_id (str): planner type
            save (bool, optional): Defaults to False. Save every runs results to CSV
            results_path (str, optional): Defaults to ''. Overwrite default results path
        
        Returns:
            results_log (dict): result_log of all queries (scenes and states)
            stats (dict): stats of the whole problem set
        """
        result_log = {}
        t_avg_run_time = 0      # Totals
        t_avg_plan_time = 0
        t_avg_dist = 0
        t_avg_path_length = 0
        t_avg_success = 0

        for x1 in xrange(len(self.scenes)):     # Scene loop
            query_count = 0                     # Initiate query count for each scene
            scene_name = self.scenes[x1]

            # Clears previous scene and loads scene to server, loads states
            scene = {'name': scene_name}        # Create scene dict
            scene_obj = Scene(self.scenes[x1])
            states = scene_obj.states

            # Loops over all states for start and target poses
            for x2 in xrange(len(states)):
                start_pose = states[x2]

                for x3 in range(x2+1, len(states)):
                    self.group.set_planner_id(planner_id)  # Set new planner ID

                    target_pose = states[x3]
                    query = {'start_pose': start_pose,
                             'target_pose': target_pose}  # Create query dict

                    rospy.loginfo('%d Executing %s from %s to %s for average over %d runs',
                                  query_count, planner_id, start_pose, target_pose, self.avg_runs)

                    # Plan path and add results to planner dict
                    planner_results = self._get_stats(start_pose, target_pose)

                    # Update query with planner results
                    query.update(planner_results)

                    t_avg_run_time += planner_results['avg_run_time']
                    t_avg_plan_time += planner_results['avg_plan_time']
                    t_avg_dist += planner_results['avg_dist']
                    t_avg_path_length += planner_results['avg_path_length']
                    t_avg_success += planner_results['avg_success']

                    if save is True:        # Append results to csv if save is true
                        with open(results_path, 'a') as f:
                            writer = csv.writer(f)
                            writer.writerow([planner_id, scene_name, query_count, start_pose, target_pose,
                                             planner_results['avg_runs'], planner_results['avg_run_time'],
                                             planner_results['avg_plan_time'], planner_results['avg_dist'],
                                             planner_results['avg_path_length'], planner_results['avg_success'],
                                             self.planner_config_obj.get_planner_params(planner_id)])

                        rospy.loginfo('avg_run_time: %.4f avg_plan_time: %.4f avg_dist: %.4f avg_path_length: %.4f avg_success: %.4f\n',
                                      planner_results['avg_run_time'],  planner_results['avg_plan_time'],
                                      planner_results['avg_dist'], planner_results['avg_path_length'],
                                      planner_results['avg_success'])

                    scene[query_count] = query      # Add query dict to scene dict
                    query_count += 1
            result_log[x1] = scene        # Add scene dict to results
            t_avg_success = t_avg_success / float(query_count)

            if save is True:
                with open(results_path, 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([''])
                    writer.writerow(['t_avg_run_time', 't_avg_plan_time',
                                     't_avg_dist', 't_avg_path_length', 't_avg_success'])
                    writer.writerow(
                        [t_avg_run_time, t_avg_plan_time, t_avg_dist, t_avg_path_length, t_avg_success])
                    writer.writerow([''])

            stats = {'t_avg_run_time': t_avg_run_time, 't_avg_plan_time': t_avg_plan_time,
                     't_avg_dist': t_avg_dist, 't_avg_path_length': t_avg_path_length, 't_avg_success': t_avg_success}

        return result_log, stats

    def _get_stats(self, start_pose, target_pose):
        run_times = []
        path_stats = []
        for _ in xrange(self.avg_runs):
            self._move_arm(start_pose)  # reset to start pose

            path = self._plan_path(start_pose, target_pose)
            run_time, exec_success = self._move_arm(
                target_pose, path['planned_path'])

            if (path['success'] == 0) or (exec_success is False):
                run_time = 0
                path = {'planned_path': 0, 'plan_time': 0, 'length': {
                    'joint_dist': 0, 'joint_length': 0}, 'success': 0}

            run_times.append(run_time)
            path_stats.append(path)

        avg_run_time = sum(run_times) / float(len(run_times))
        avg_plan_time = float(sum(d['plan_time']
                                  for d in path_stats)) / len(path_stats)
        avg_dist = float(sum(d['length']['joint_dist']
                             for d in path_stats)) / len(path_stats)
        avg_path_length = float(sum(d['length']['joint_length']
                                    for d in path_stats)) / len(path_stats)
        avg_success = float(sum(d['success']
                                for d in path_stats)) / len(path_stats)

        # To penalise aborted plans which are missed (t_avg_success=1 but execution plan aborted (plan time <= 50ms)).
        # This only happens with BKPIECEkConfigDefault planner
        if (avg_success == 1) and (avg_plan_time <= 0.01):
            avg_success = 0

        return {'avg_runs': self.avg_runs, 'avg_run_time': avg_run_time, 'avg_plan_time': avg_plan_time,
                'avg_dist': avg_dist, 'avg_path_length': avg_path_length, 'avg_success': avg_success}

    def _plan_path(self, start_pose, target_pose):
        # start_state = self.robot.get_current_state()
        # start_state.joint_state.position = start_pose
        # self.group.set_start_state(start_state)
        self.group.set_named_target(target_pose)
        start_time = timer()
        planned_path = self.group.plan()
        plan_time = timer() - start_time

        # If motion plan fails, length will be saved as 0
        length = {'joint_dist': 0, 'joint_length': 0}
        success = 0
        if len(planned_path.joint_trajectory.points) != 0:
            length = self._get_path_length(planned_path)
            # plan_time_secs = planned_path.joint_trajectory.points[-1].time_from_start.secs
            # plan_time_nsecs = planned_path.joint_trajectory.points[-1].time_from_start.nsecs
            # plan_time = plan_time_secs + plan_time_nsecs*1e-9
            if length['joint_length'] >= 0.001:
                success=1

        return {'planned_path': planned_path, 'plan_time': plan_time, 'length': length, 'success': success}

    def _get_path_length(self, path):
        pts = path.joint_trajectory.points
        j_length = 0    # j = jointspace
        # w_length = 0    # w = workspace
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
    #     rospy.wait_for_service('compute_fk')
    #     try:
    #         moveit_fk = rospy.ServiceProxy(
    #             'compute_fk', moveit_msgs.srv._GetPositionFK.GetPositionFK)
    #     except rospy.ServiceException, e:
    #         rospy.logerror('Service call failed: %s' % e)

    #     fkln = ['ee_link']  # forward kinematic link to be calculated

    #     # make header for the argument to moveit_fk
    #     header = std_msgs.msg.Header(0, rospy.Time.now(), '/world')

    #     rs = self.robot.get_current_state()
    #     rs.joint_state.position = joint_pos  # robot state for argument to moveit_fk

    #     fwd_kin = moveit_fk(header, fkln, rs)

    #     print(fwd_kin)
    #     try:
    #         pos = fwd_kin.pose_stamped[0].pose.position  # extract position
    #         fwd_kin_coordinates = [pos.x, pos.y, pos.z]
    #     except IndexError:
    #         fwd_kin_coordinates = [0, 0, 0]

    #     return fwd_kin_coordinates

    def _move_arm(self, pose, plan=None):
        self.group.set_named_target(pose)

        if plan is None:    # When moving back to start
            plan = self.group.plan()

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        start_time = timer()
        success = self.group.execute(plan, wait=True)
        run_time = timer() - start_time

        return run_time, success

    # def get_results(self):
    #     # try:
    #     # catch:

    #     results = pd.read_csv(self.results_path)
    #     # Sort with best scores on top and reset index for slicing

    #     for p in results.planner.unique():
    #         planner_df = results[results['planner'] == p]
    #         #print(planner_df)
    #         planner_df.sort_values(
    #             'loss', ascending=True, inplace=True)
    #         planner_df.reset_index(inplace=True, drop=True)
    #         print(planner_df.head())
