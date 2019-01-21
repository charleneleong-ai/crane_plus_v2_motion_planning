#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 7:18:59 pm
# Author: Charlene Leong
# Last Modified: Monday, January 21st 2019, 2:53:18 pm
# Modified By: Charlene Leong
###

from timeit import default_timer as timer
import os
import time
import csv
import pprint
import pandas as pd
import numpy as np

import rospkg
import rospy
import std_msgs.msg

import moveit_commander
import moveit_msgs.msg

from planner_config import PlannerConfig
from scene_object import Scene

ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')

# from objectives import objectives


class Session(object):
    """
    Session Base Class
    """
    def __init__(self, mode):
        self.planner_config_obj = PlannerConfig()
        self.mode = mode
        self.planner_config = self.planner_config_obj.planner_config
        self.planners = self.planner_config_obj.planners

        self.n_trial = 0
        if self.mode not in ['default', 'ompl']:
            self.max_trials = rospy.get_param('~max_trials')
        self.avg_runs = rospy.get_param('~avg_runs')
        self.start_pose = self.planner_config_obj.start_pose
        self.target_pose = self.planner_config_obj.target_pose

        self.results_path = ROS_PKG_PATH+'/results/' + \
            self.planner_config_obj.planner_select+'_'+self.mode+'.csv'

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.planning_frame = self.group.get_planning_frame()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)
        self.scenes = ['narrow_passage']

    def run(self):
        raise NotImplementedError

    def _run_problem_set(self, planner_id, save=False, results_path=""):
        result_log = {}
        t_avg_run_time = 0      # Totals
        t_avg_plan_time = 0
        t_avg_dist = 0
        t_avg_path_length = 0
        t_avg_success = 0

        for x1 in xrange(len(self.scenes)):     # Scene loop
            query_count = 0                    # Initiate query count for each scene
            scene_name = self.scenes[x1]

            # Clears previous scene and loads scene to server, loads states
            scene_obj = Scene(self.scenes[x1])
            states = scene_obj.states

            scene = {'name': scene_name}         # Create scene dict

            # Loops over all states for start and target poses
            for x2 in xrange(len(states)):
                start_pose = states[x2]

                for x3 in range(x2+1, len(states)):
                    target_pose = states[x3]

                    query = {'start_pose': start_pose,
                             'target_pose': target_pose}  # Create query dict

                    self.group.set_planner_id(planner_id)  # set new planner ID

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

                        rospy.loginfo('avg_run_time: %.4f avg_plan_time: %.4f avg_dist: %4f avg_path_length: %.4f avg_success: %.4f\n',
                                      planner_results['avg_run_time'],  planner_results['avg_plan_time'],
                                      planner_results['avg_dist'], planner_results['avg_path_length'],
                                      planner_results['avg_success'])

                    # Add query dict to scene dict
                    scene[query_count] = query
                    query_count += 1
            result_log[x1] = scene        # Add scene dict to results
            t_avg_success = t_avg_success / float(query_count)

            stats = {'t_avg_run_time': t_avg_run_time, 't_avg_plan_time': t_avg_plan_time,
                     't_avg_dist': t_avg_dist, 't_avg_path_length': t_avg_path_length, 't_avg_success': t_avg_success}

            if save is True:
                with open(results_path, 'a') as f:
                    writer = csv.writer(f)
                    writer.writerow([''])
                    writer.writerow(
                        ['t_avg_run_time', 't_avg_plan_time', 't_avg_dist', 't_avg_success'])
                    writer.writerow([stats['t_avg_run_time'], stats['t_avg_plan_time'],
                                     stats['t_avg_dist'], stats['t_avg_success']])
                    writer.writerow([''])

        return result_log, stats

    def _get_stats(self, start_pose, target_pose):
        run_times = []
        path_stats = []
        for _ in xrange(self.avg_runs):
            self._move_arm(start_pose)  # reset to start pose
        
            path = self._plan_path(start_pose, target_pose)
            run_time, exec_success = self._move_arm(target_pose, path['planned_path'])

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

        return {'avg_runs': self.avg_runs, 'avg_run_time': avg_run_time, 'avg_plan_time': avg_plan_time,
                'avg_dist': avg_dist, 'avg_path_length': avg_path_length, 'avg_success': avg_success}

    def _plan_path(self, start_pose, target_pose):
        """Returns path (RobotTrajectory) given a start pose and target pose
        
        Args:
            start_pose (str): desired start pose
            target_pose (str): desired target psoe
        
        Returns:
            (dict{str}): path info {path, planning time, path length}
        """
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
            success = 1
        
        return {'planned_path': planned_path, 'plan_time': plan_time, 'length': length, 'success': success}
    
    def _get_path_length(self, path): 
        """Returns the eucld dist and path length of given motion plan (RobotTrajectory)
        Args:
            path (RobotTrajectory) -- MoveIt RobotTrajectory Message
        
        Returns:
            (dict{str}) -- eucld dist of path and path length in jointspace
        """
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

        if plan is None:
            plan = self.group.plan()

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)
        start_time = timer()
        success = self.group.execute(plan, wait=True)
        run_time = timer() - start_time
        
        return run_time, success

    def get_results(self):
        # try:
        # catch:

        results = pd.read_csv(self.results_path)
        # Sort with best scores on top and reset index for slicing

        for p in results.planner.unique():
            planner_df = results[results['planner'] == p]
            #print(planner_df)
            planner_df.sort_values(
                'loss', ascending=True, inplace=True)
            planner_df.reset_index(inplace=True, drop=True)
            print(planner_df.head())
