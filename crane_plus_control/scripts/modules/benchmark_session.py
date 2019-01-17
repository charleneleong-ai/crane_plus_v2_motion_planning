#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 2:10:34 pm
# Modified By: Charlene Leong
# Last Modified: Thursday, January 17th 2019, 1:32:45 pm
# Author: Charlene Leong (charleneleong84@gmail.com)
###

import sys
import os
import time
import csv
import json

import numpy
import cPickle as pickle
import pprint
import json

import rospkg
import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs
import shape_msgs
import std_msgs
from datetime import datetime as dt

from session import Session
from scene_object import Scene

ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/scripts'


class BenchmarkSession(Session):
    """Constructor for benchmarking session

    Args:
        Session (object): Session object initiates session with default functions
    """

    PLANNING_TIME = 2  # second,

    def __init__(self, mode):
        super(BenchmarkSession, self).__init__(mode)
        rospy.loginfo('Initialising benchmarking session')
        self.scenes = ['narrow']
        # self.states = rospy.get_param('~named_states')

    def run(self):
        self.group.set_planning_time(self.PLANNING_TIME)

        with open(self.results_path, 'w') as f:
            writer = csv.writer(f)
            writer.writerow(['scene', 'query', 'planner', 'start_pose', 'target_pose', 'avg_runs', 'avg_run_time',
                             'avg_plan_time', 'avg_dist', 'avg_path_length', 'params'])

        self.results = {}                       # Create empty results dict
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
                        
                        planner_results = super(BenchmarkSession, self)._get_stats(
                            start_pose, target_pose)    # Plan path and add results to planner dict

                        # Add planner results to query dict
                        query[planner_name] = planner_results

                        # Append results to csv
                        with open(self.results_path, 'a') as f:
                            writer = csv.writer(f)
                            writer.writerow([scene_name, query_count, planner_name, start_pose, target_pose, planner_results['avg_runs'],
                                            planner_results['avg_run_time'], planner_results['avg_plan_time'], planner_results['avg_dist'],
                                            planner_results['avg_path_length'], self.planner_config_obj.get_planner_params(planner_name)])


                    # Add query dict to scene dict
                    scene['query'] = query
                    self.results['scene'] = scene

                    query_count += 1

        # data_string = ROS_PKG_PATH+'/benchmark_' + str(dt.now().year) + '.' + str(dt.now().month) + '.' + str(
        #     dt.now().day) + '_' + str(dt.now().hour) + '.' + str(dt.now().minute) + '_' + str(self.iter) + '.p'

        # with open(data_string, 'wb') as fp:    # Dump with run
        #     pickle.dump(self.results, fp)

        # Dump as latest benchmark
        with open(ROS_PKG_PATH+'/benchmark_latest.p', 'wb') as fp:
            pickle.dump(self.results, fp)

        pprint.pprint(self.results)
