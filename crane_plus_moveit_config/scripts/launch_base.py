#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 2:03:37 pm
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 23rd 2019, 8:40:48 pm
# Author: Charlene Leong (charleneleong84@gmail.com)
###

import sys
import rospkg
import rospy
import moveit_commander
import geometry_msgs.msg

ROS_PKG_PATH =  rospkg.RosPack().get_path('crane_plus_moveit_config') + '/scripts'

class Base(object):
    def __init__(self):
        
        self.group = moveit_commander.MoveGroupCommander('arm')
        self.robot = moveit_commander.RobotCommander()
        self.planning_frame = self.robot.get_planning_frame()
        self.scene = moveit_commander.PlanningSceneInterface()
        self._load_base_scene()
        
    def _load_base_scene(self):
        with open(ROS_PKG_PATH+'/base.scene') as f:
            title = f.readline()
            self.name = f.readline()[2:]
            number = f.readline()     # object number?
            shape = f.readline()      # object shape
            
            #****** Parsing dimension ******#
            text = f.readline()
            self.dim = []
            for x in range (0, 3):          #3D dimension
                loc = text.find(' ')
                self.dim.append(float(text[:loc]))
                text = text[loc+1:]         #Remove used text

            #****** Parsing Location ******#
            text = f.readline()
            self.pos = []
            for x in range (0, 3):          #3D dimension
                loc = text.find(' ')        
                self.pos.append(float(text[:loc]))
                text = text[loc+1:]
            

    def wait_for_state_update(self, base_is_known=False, base_is_attached=False, timeout=4):
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the base is in attached objects
            attached_objects = self.scene.get_attached_objects([self.name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the base is in the scene.
            # Note that attaching the base will remove it from known_objects
            is_known = self.name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (base_is_attached == is_attached) and (base_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.01)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False


    def add_base(self, timeout=4):
        rospy.sleep(4)
        pose = geometry_msgs.msg.PoseStamped()
        pose.header.frame_id = self.planning_frame
        pose.pose.position.x = self.pos[0]
        pose.pose.position.y = self.pos[1]
        pose.pose.position.z = self.pos[2]
        self.scene.add_box(self.name, pose, (self.dim[0], self.dim[1], self.dim[2]))
       
        return self.wait_for_state_update(base_is_known=True, timeout=timeout)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('launch_base', anonymous=True)
    base = Base()
    base.add_base()
   
if __name__ == '__main__':
    main()
