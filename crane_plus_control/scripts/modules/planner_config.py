#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 10:02:10 am
# Modified By: charlene
# Last Modified: Wed Jan 16 2019
# Author: Charlene Leong (charleneleong84@gmail.com)
###


import sys
import rospy

from moveit_msgs.srv import GetPlannerParams, SetPlannerParams

class PlannerConfig(object):
    def __init__(self):
        self.planning_time = 2  # seconds

        self.planner_select = rospy.get_param(
            "~planner_config")
        if self.planner_select not in ['Cano_etal']:
                rospy.logerr("Invalid planner config select")
                sys.exit(1)

        # self.start_pose = rospy.get_param("~start_pose")
        # self.target_pose = rospy.get_param("~target_pose")
        self.named_states = rospy.get_param("~named_states")

        # if self.target_pose not in self.named_states:
        #     rospy.logerr('target_pose not in list of named_states')
        #     rospy.logerr(self.named_states)
        #     sys.exit(1)
        # elif self.start_pose not in self.named_states:
        #     rospy.logerr('start_pose not in list of named_states')
        #     rospy.logerr(self.named_states)
        #     sys.exit(1)

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

        self.planners = list(self.planner_config.keys())
        self.name = self.planner_select+"_"+self.mode

    def get_planner_config(self):
        return self.planner_config
        
    def get_planner_mode(self):
        return self.mode

    def get_planners(self):
        return self.planners

    def get_planner_params(self, planner_id):
        # rospy.loginfo('Waiting for get_planner_params')
        rospy.wait_for_service('get_planner_params')
        get_planner_params = rospy.ServiceProxy(
            'get_planner_params', GetPlannerParams)
        try:
            req = get_planner_params(planner_id, "arm")
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)

        # params = {}
        # for k in len(req.params.keys):
        #     params[req.params.keys]
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

    
