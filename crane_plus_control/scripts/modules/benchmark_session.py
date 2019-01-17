#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 2:10:34 pm
# Modified By: charlene
# Last Modified: Thursday, January 17th 2019, 10:10:07 am
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

ROS_PKG_PATH =  rospkg.RosPack().get_path('crane_plus_control')+'/scripts'

# from objectives import objectives
class BenchmarkSession(Session):
    """Constructor for benchmarking session
    
    Args:
        Session (object): Session object initiates session with default functions
    """

    PLANNING_TIME = 2 # second, 

    def __init__(self):
        super(BenchmarkSession, self).__init__()
        rospy.loginfo("Initialising benchmarking session")
        self.scenes = ["narrow"]
        # self.states = rospy.get_param('~named_states')

    def run(self):
        """Runs benchmarking session
        """

        prog_counter = 0   			# Progression counter that counts to 2*amount of scenes

        self.group.set_planning_time(self.PLANNING_TIME)     

        self.results = {}                       # Create empty results dict
        for x1 in xrange(len(self.scenes)):     # Scene loop           
            scene_name = self.scenes[x1]

            scene_obj = Scene(self.scenes[x1])           # Clears previous scene and loads scene to server, loads states
            states = scene_obj.states                    # Gets loaded states

            scene = {"name":scene_name}         # Create scene dict
            query_count = 0             
            for x2 in xrange(len(states)):

                start = states[x2]
                for x3 in range (x2+1, len(states)):         # Loops to decide start and goal states
                        
                    goal = states[x3]
                                        
                    query = {"start":start, "goal":goal}        # Create query dict
                    
                    for x4 in xrange(len(self.planners)):                        
                        planner_name = self.planners[x4]         
                      
                        planner_results = super(BenchmarkSession, self)._get_stats(start,goal)    # Plan path and add results to planner dict    
                        query[planner_name] = planner_results                         # Add planner results to query dict
                    scene["query"] = query                  # Add query dict to scene dict
                    # query_count += 1
                    self.results["scene"] = scene  
		    prog_counter += 1
		    
		    print (prog_counter)
            #self.results[x1] = scene                            # Add scene dict to results dict          

        # data_string = ROS_PKG_PATH+'/benchmark_' + str(dt.now().year) + '.' + str(dt.now().month) + '.' + str(dt.now().day) + '_' + str(dt.now().hour) + '.' + str(dt.now().minute) + '_' + str(self.iter) + '.p'
       
        # with open(data_string, 'wb') as fp:    # Dump with run
        #     pickle.dump(self.results, fp)

        with open(ROS_PKG_PATH+'/benchmark_latest.p', 'wb') as fp:    # Dump as latest benchmark
            pickle.dump(self.results, fp)

        pprint.pprint(self.results)

        
        # json = json.dumps(self.results)

