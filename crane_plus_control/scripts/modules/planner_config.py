#!/usr/bin/env python
###
# File Created: Wednesday, January 16th 2019, 7:18:59 pm
# Author: Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 11:02:12 am
###

import sys
import rospy
import moveit_msgs.msg
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams


class PlannerConfig(object):
    """
    PlannerConfig Object
    set_planner_params(planner_id, params_set): Sets planner params on the ROS Param server
    get_planner_params(planner_id): Gets planner params from the ROS Param server
    """

    def __init__(self):
        self.PLANNER_SELECT = rospy.get_param('~planner_select')
        self.PLANNER = rospy.get_param('~planner')
        if rospy.get_param('~mode') in ['default', 'ompl']:
            self.NAME = self.PLANNER_SELECT+'_default'
            self.planner_config = rospy.get_param('~planner_configs_'+self.NAME)
                
        else:
            self.NAME = self.PLANNER_SELECT+'_tune'
            self.planner_config = rospy.get_param('~planner_configs_'+self.NAME)
        assert isinstance(self.planner_config, dict)
        

        # self.planners = list(self.PLANNER)
        if self.PLANNER == 'all':
            self.planners = self.planner_config.keys()
        else:
            self.planners = [self.PLANNER+'kConfigDefault']

        # Override planner config with OMPL config in ompl mode
        if rospy.get_param('~mode') == 'ompl':
            self.NAME = self.PLANNER_SELECT+'_ompl'
            ompl = rospy.get_param('/move_group/planner_configs/')
            for p in self.planner_config.keys():
                self.planner_config[p] = ompl[p]

        for k, v in self.planner_config.iteritems():
            self.set_planner_params(k, v)

    def set_planner_params(self, planner_id, params_set):
        """Calls SetPlannerParams moveit ROS service to set planner params for select planner
        
        Args:
            planner_id (str): planner name
            params_set (dict): planner params
        """
        # Convert dict to PlannerParams msg
        params = moveit_msgs.msg.PlannerParams()
        params.keys = params_set.keys()
        params.values = [str(v) for v in params_set.values()]

        rospy.wait_for_service('set_planner_params')
        set_planner_params = rospy.ServiceProxy(
            'set_planner_params', SetPlannerParams)
        try:
            set_planner_params(planner_id, 'arm', params, True)
            rospy.loginfo('%s %s parameters updated', self.PLANNER_SELECT, planner_id)
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)

    def get_planner_params(self, planner_id):
        """Calls GetPlannerParams moveit ROS service and returns params for select planenr
        
        Arguments:
            planner_id {str} -- planner name
        
        Returns:
            dict -- planner params
        """
        rospy.wait_for_service('get_planner_params')
        get_planner_params = rospy.ServiceProxy(
            'get_planner_params', GetPlannerParams)
        try:
            req = get_planner_params(planner_id, 'arm')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)

        # Convert PlannerParams msg to dict
        params = {}
        for idx, k in enumerate(req.params.keys):
            params[k] = req.params.values[idx]
        return params

    def get_planner_config(self):
        return self.planner_config

    def get_planner_config_name(self):
        return self.NAME

    def get_planners(self):
        return self.planners
