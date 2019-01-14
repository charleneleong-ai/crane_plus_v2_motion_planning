#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from parameter_tuning_session import ParamTuningSession

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning', anonymous=True)
    session = ParamTuningSession()
    session.run()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
