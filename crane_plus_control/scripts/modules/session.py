#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Tuesday, 15th January 2019 3:19:44 pm
# Modified By: Charlene Leong
# Last Modified: Thursday, January 17th 2019, 3:22:50 pm
# Author: Charlene Leong (charleneleong84@gmail.com)
###

from timeit import default_timer as timer
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
        self.iter = rospy.get_param('~iter')
        self.start_pose = self.planner_config_obj.start_pose
        self.target_pose = self.planner_config_obj.target_pose

        self.results_path = ROS_PKG_PATH+'/results/'+self.planner_config_obj.planner_select+'_'+self.mode+'.csv'

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.planning_frame = self.group.get_planning_frame()  
        self.scene = moveit_commander.PlanningSceneInterface()
        self.display_trajectory_publisher = rospy.Publisher('/group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)

        self.scenes = ['narrow']

    def _move_arm(self, pose, plan=None):
        
        self.group.set_named_target(pose)

        if plan is None:
            plan = self.group.plan()
        
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        self.display_trajectory_publisher.publish(display_trajectory)

        #rospy.loginfo('Moving to %s pose', pose)
        self.group.go(wait=True)
        self.group.stop()

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

        length = 0
        if len(planned_path.joint_trajectory.points) != 0:  # not sure how to figure out a failure otherwise
            length = self._get_path_length(planned_path)

        return {'planned_path': planned_path, 'plan_time': plan_time, 'length': length}

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

    def _get_forward_kinematics(self, joint_pos):
        rospy.wait_for_service('compute_fk')
        try:
            moveit_fk = rospy.ServiceProxy(
                'compute_fk', moveit_msgs.srv._GetPositionFK.GetPositionFK)
        except rospy.ServiceException, e:
            rospy.logerror('Service call failed: %s' % e)

        fkln = ['ee_link']  # forward kinematic link to be calculated

        # make header for the argument to moveit_fk
        header = std_msgs.msg.Header(0, rospy.Time.now(), '/world')

        rs = self.robot.get_current_state()
        rs.joint_state.position = joint_pos  # robot state for argument to moveit_fk

        fwd_kin = moveit_fk(header, fkln, rs)

        print(fwd_kin)
        try:
            pos = fwd_kin.pose_stamped[0].pose.position  # extract position
            fwd_kin_coordinates = [pos.x, pos.y, pos.z]
        except IndexError:
            fwd_kin_coordinates = [0, 0, 0]

        return fwd_kin_coordinates

    def _get_stats(self, start_pose, target_pose):
        run_times = []
        path_stats = []
        for _ in xrange(self.iter):
            self._move_arm(start_pose)  # reset to start pose
            start_time = timer()
            path = self._plan_path(start_pose, target_pose)
            self._move_arm(target_pose, path['planned_path'])
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
        result = { 'avg_runs': self.iter, 'avg_run_time': avg_run_time, 'avg_plan_time': avg_plan_time,
                  'avg_dist': avg_dist, 'avg_path_length': avg_path_length}

        return result
   
    def _run_problem_set(self):
        if self.mode in ['default', 'ompl']:
            with open(self.results_path, 'w') as f:
                writer = csv.writer(f)
                writer.writerow(['scene', 'query', 'planner', 'start_pose', 'target_pose', 'avg_runs', 'avg_run_time',
                                'avg_plan_time', 'avg_dist', 'avg_path_length', 'params'])

        results = {}                       # Create empty results dict
        for x1 in xrange(len(self.scenes)):     # Scene loop
            scene_name = self.scenes[x1]

            # Clears previous scene and loads scene to server, loads states
            scene_obj = Scene(self.scenes[x1])
            states = scene_obj.states                    # Gets loaded states

            query_count = 1
            scene = {'name': scene_name}         # Create scene dict
            scene['query_count'] = query_count
            
            for x2 in xrange(len(states)):

                start_pose = states[x2]

                # Loops to decide start_pose and target_pose states
                for x3 in range(x2+1, len(states)):

                    target_pose = states[x3]

                    query = {'start_pose': start_pose,
                             'target_pose': target_pose}  # Create query dict

                    for x4 in xrange(len(self.planners)):
                        planner_name = self.planners[x4]
                        self.group.set_planner_id(planner_name)     #set new planner ID

                        rospy.loginfo('%d Executing %s from %s to %s for average over %d runs',
                            query_count, planner_name, start_pose, target_pose, self.iter)
                        
                        planner_results = self._get_stats(
                            start_pose, target_pose)    # Plan path and add results to planner dict
                        
                        query[planner_name] = planner_results   # Add planner results to query dict

                        # Append results to csv if in benchmark session
                        if self.mode in ['default', 'ompl']:
                            with open(self.results_path, 'a') as f:
                                writer = csv.writer(f)
                                writer.writerow([scene_name, query_count, planner_name, start_pose, target_pose, planner_results['avg_runs'],
                                                planner_results['avg_run_time'], planner_results['avg_plan_time'], planner_results['avg_dist'],
                                                planner_results['avg_path_length'], self.planner_config_obj.get_planner_params(planner_name)])

                    scene['query'] = query      # Add query dict to scene dict
                    results['scene'] = scene

                    query_count += 1

        # data_string = ROS_PKG_PATH+'/benchmark_' + str(dt.now().year) + '.' + str(dt.now().month) + '.' + str(dt.now().day) + '_' + str(dt.now().hour) + '.' + str(dt.now().minute) + '_' + str(self.iter) + '.p'
       
        # with open(data_string, 'wb') as fp:    # Dump with run
        #     pickle.dump(results, fp)
            
        rospy.loginfo('Saved results to %s', self.results_path)

        return results

    def run(self):
        raise NotImplementedError, 'Should be implemented in child class'

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