#!/usr/bin/env python
'''
File Created: Wednesday, 16th January 2019 8:49:00 am
Last Modified: Wednesday, 16th January 2019 10:02:20 am
Author: Charlene Leong (charleneleong84@gmail.com)
'''

import sys
import rospy
import moveit_commander
from modules.parameter_tuning_session import ParamTuningSession
from modules.benchmark_session import BenchmarkSession

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('parameter_tuning', anonymous=True)
    # session = ParamTuningSession()
    session = BenchmarkSession()
    session.run()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
