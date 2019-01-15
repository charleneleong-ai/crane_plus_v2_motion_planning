#!/usr/bin/env python

import sys
import copy
import rospkg
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list




ROS_PKG_PATH =  rospkg.RosPack().get_path('crane_plus_moveit_config') + '/scripts'


class Base(object):
    def __init__(self):
        
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.robot = moveit_commander.RobotCommander()
        self.planning_frame = self.robot.get_planning_frame()
        self.scene = moveit_commander.PlanningSceneInterface()

        self._load_base_scene()
        
    def _load_base_scene(self):
        with open(ROS_PKG_PATH+"/base.scene") as f:
            title = f.readline()
            self.name = f.readline()[2:]
            number = f.readline()     # object number?
            shape = f.readline()      # object shape
            
            #****** Parsing dimension ******#
            text = f.readline()
            self.dim = []
            for x in range (0, 3):          #3D dimension
                loc = text.find(" ")
                self.dim.append(float(text[:loc]))
                text = text[loc+1:]         #Remove used text

            #****** Parsing Location ******#
            text = f.readline()
            self.pos = []
            for x in range (0, 3):          #3D dimension
                loc = text.find(" ")        
                self.pos.append(float(text[:loc]))
                text = text[loc+1:]
            
            # #****** Parsing Rotation ******#
            # text = f.readline()
            # self.rot = []
            # for x in range (0, 4):          #4D dimension
            #     loc = text.find(" ")
            #     self.rot.append(float(text[:loc]))
            #     text = text[loc+1:]
                
            # #****** Parsing Colour ******#
            # text = f.readline()
            # self.col = []
            # for x in range (0, 4):
            #     loc = text.find(" ")
            #     self.col.append(float(text[:loc]))
            #     text = text[loc+1:]
            # # Currently unused, also not needed for adding objects

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
        rospy.sleep(2)
        p = geometry_msgs.msg.PoseStamped()
        p.header.frame_id = self.planning_frame
        p.pose.position.x = self.pos[0]
        p.pose.position.y = self.pos[1]
        p.pose.position.z = self.pos[2]
        self.scene.add_box("box", p, (self.dim[0], self.dim[1], self.dim[2]))

        return self.wait_for_state_update(box_is_known=True, timeout=timeout)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('launch_base', anonymous=True)
    base = Base()
    base.add_base()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
