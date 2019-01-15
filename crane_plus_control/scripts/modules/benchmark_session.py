#!/usr/bin/env python

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

ROS_PKG_PATH =  rospkg.RosPack().get_path('crane_plus_control')

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
        self.scenes = ["scene_7", "scene_6"]

        #Object Publishers, can alsu use PlanningSceneInterface, but this doesn throw any warnings
        self.object_publisher = rospy.Publisher('/collision_object',
                moveit_msgs.msg._CollisionObject.CollisionObject,
                queue_size=100)

        self.attached_object_publisher = rospy.Publisher('/attached_collision_object',
                moveit_msgs.msg._AttachedCollisionObject.AttachedCollisionObject,
            reboot    queue_size=30)

        self.env_names = []                   

        self.states = rospy.get_param('~named_states')
        #print(self.planners)

    def _load_env(self, scene):
        """ 
        Function that parses a .scene file and loads the environment. Currently
        Only supports the BOX type, but can be easily extended if needed
        """   

        with open(ROS_PKG_PATH+"/scripts/scenes/"+scene+".scene") as f:
            
            lines = 0   # get lines to know amount of blocks
            for line in f:
                lines += 1                
            f.seek(0)   # reset readfile location
               
            title = f.readline()
            rospy.loginfo("Loading scene:%s" % title)
            
            cnt = (lines-2)/7             # object data is 7 lines long, count amount of objects
            
            for objs in range (0, cnt):
                name = f.readline()[2:]   # object name
                number = f.readline()     # object number?
                shape = f.readline()      # object shape
                
                self.env_names.append(name)   #adding name to list in order to clear later
                
                #****** Parsing dimension ******#
                text = f.readline()
                dim = []
                for x in range (0, 3):          #3D dimension
                    loc = text.find(" ")
                    dim.append(float(text[:loc]))
                    text = text[loc+1:]         #Remove used text
                
                #****** Parsing Location ******#
                text = f.readline()
                pos = []
                for x in range (0, 3):          #3D dimension
                    loc = text.find(" ")        
                    pos.append(float(text[:loc]))
                    text = text[loc+1:]
                    
                #****** Parsing Rotation ******#
                text = f.readline()
                rot = []
                for x in range (0, 4):          #4D dimension
                    loc = text.find(" ")
                    rot.append(float(text[:loc]))
                    text = text[loc+1:]
                    
                #****** Parsing Colour ******#
                text = f.readline()
                col = []
                for x in range (0, 4):
                    loc = text.find(" ")
                    col.append(float(text[:loc]))
                    text = text[loc+1:]
                # Currently unused, also not needed for adding objects
                    
                
                #******* adding the object ********#
                object_shape = shape_msgs.msg._SolidPrimitive.SolidPrimitive()
                object_shape.type = object_shape.BOX #extend support for other primitives?
                object_shape.dimensions = dim
                
                object_pose = geometry_msgs.msg._Pose.Pose()
                object_pose.position.x = pos[0]
                object_pose.position.y = pos[1]
                object_pose.position.z = pos[2]
                
                object_pose.orientation.x = rot[0]
                object_pose.orientation.y = rot[1]
                object_pose.orientation.z = rot[2]
                object_pose.orientation.w = rot[3]
                
                object = moveit_msgs.msg.CollisionObject()
                object.id = name
                object.header.frame_id = self.planning_frame
                object.primitives.append(object_shape)
                object.primitive_poses.append(object_pose)
                
                assert type(object) == moveit_msgs.msg.CollisionObject
                
                object.header.stamp = rospy.Time.now()
                object.operation = object.ADD
                self.object_publisher.publish(object)
                time.sleep(self.PUBLISHER_DELAY)

            scene.

    
    def _clear_env(self):        
        """  
        Clears the collision model for a new scene to be loaded 
        """    
        for x in xrange(len(self.env_names)):
            # print (self.env_names[x])
            self.scene.remove_world_object(self.env_names[x])
        self.env_names = []

    def run(self):
        
        prog_counter = 0   			#progression counter that counts to 2*amount of scenes

        self.group.set_planning_time(self.PLANNING_TIME)     

        self.results = {}                       #create empty results dict
        for x1 in xrange(len(self.scenes)):     #scene loop            
    	    
            # if (os.path.isfile('/home/ruben/moveit_ompl/new.db')):
    	    #     os.remove('/home/ruben/moveit_ompl/new.db') # remove the new database in case of thunder
               
            scene_name = self.scenes[x1]

            self._load_env(scene_name)           #load environment into collision model
       
            scene = {"name":scene_name}         #create scene dict
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
    	    
            with open(ROS_PKG_PATH+'/scripts/benchmark_data.p', 'wb') as fp:    #store this scene's data
                	pickle.dump(self.results, fp)                
            self._clear_env()

        data_string = ROS_PKG_PATH+'/scripts/bm_' + str(dt.now().month) + '.' + str(dt.now().day) + '_' + str(dt.now().hour) + '.' + str(dt.now().minute) + '_' + str(self.iter) + '.p'
    
        # with open(data_string, 'wb') as fp:    #final data store
        #     pickle.dump(self.results, fp)

        # with open(ROS_PKG_PATH+'/scripts/benchmark_data.p', 'wb') as fp:    #final data store
        #     pickle.dump(self.results, fp)

        pprint.pprint(self.results)
        #print(json.dumps(self.results))