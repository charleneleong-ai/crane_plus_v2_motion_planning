#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 2:10:34 pm
# Modified By: charlene
# Last Modified: Wed Jan 16 2019
# Author: Charlene Leong (charleneleong84@gmail.com)
###


import sys
import os
import time

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
    
    PUBLISHER_DELAY = 0.02 # max. 50 topics per second
    PLANNING_TIME = 2 # second, 

    def __init__(self):
        super(BenchmarkSession, self).__init__()
        rospy.loginfo("Initialising benchmarking session")
        # self.planner_config_obj = PlannerConfig()
        # self.mode = self.planner_config_obj.mode
        # self.name = self.planner_config_obj.name
        # self.planner_config = self.planner_config_obj.planner_config
        # self.planners = self.planner_config_obj.planners

        # self.n_trial = 0
        # if self.mode != "baseline":
        #     self.max_trials = rospy.get_param("~max_trials")
        # self.iter = rospy.get_param("~iter")
        # self.start_pose = self.planner_config_obj.start_pose
        # self.target_pose = self.planner_config_obj.target_pose

        # self.results_path = ROS_PKG_PATH+'/results/'+self.name+".csv"

        # self.robot = moveit_commander.RobotCommander()
        # self.group = moveit_commander.MoveGroupCommander("arm")
        # # self.planning_frame = self.group.get_planning_frame()  # "/world"
        # # self.scene = moveit_commander.PlanningSceneInterface()

        #Configuration settings
        self.scenes = [ "narrow"]

        self.states = rospy.get_param('~named_states')


    def run(self):
        prog_counter = 0   			# Progression counter that counts to 2*amount of scenes

        self.group.set_planning_time(self.PLANNING_TIME)     

        self.results = {}                       # Create empty results dict
        for x1 in xrange(len(self.scenes)):     # Scene loop           
            scene_name = self.scenes[x1]

            Scene(self.scenes[x1])           # Load scene 
            
            scene = {"name":scene_name}         # create scene dict
            query_count = 0             
            for x2 in xrange(len(self.states)):

                start = self.states[x2]
                for x3 in range (x2+1, len(self.states)):         #loops to decide start and goal states
                        
                    goal = self.states[x3]
                                        
                    query = {"start":start, "goal":goal}        #create query dict
                    
                    for x4 in xrange(len(self.planners)):                        
                        planner_name = self.planners[x4]         
                        self.group.set_planner_id(planner_name)     #set new planner ID
                                      
                        #planner_results = {"name":planner_name}     #create new planner dict                   
                        planner_results = super(BenchmarkSession, self)._get_stats(start,goal)    #plan path and add results to planner dict    
                        query[planner_name] = planner_results                         #add planner dict to query dict
                    scene[query_count] = query                  #add query dict to scene dict
                    query_count += 1
		    prog_counter += 1
		    
		    print (prog_counter)
                    
            self.results[x1] = scene                            #add scene dict to results dict
    	    
                   
            

        data_string = ROS_PKG_PATH+'/benchmark_' + str(dt.now().year) + '.' + str(dt.now().month) + '.' + str(dt.now().day) + '_' + str(dt.now().hour) + '.' + str(dt.now().minute) + '_' + str(self.iter) + '.p'
       
        with open(data_string, 'wb') as fp:    # Dump with run
            pickle.dump(self.results, fp)

        with open(ROS_PKG_PATH+'/benchmark_latest.p', 'wb') as fp:    # Dump as latest benchmark
            pickle.dump(self.results, fp)


        pprint.pprint(self.results)
        #print(json.dumps(self.results))