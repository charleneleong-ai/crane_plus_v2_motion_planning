#!/usr/bin/env python
# -*- coding:utf-8 -*-
###
# File Created: Wednesday, 16th January 2019 1:56:33 pm
# Modified By: Charlene Leong
# Last Modified: Thursday, January 17th 2019, 10:14:58 am
# Author: Charlene Leong (charleneleong84@gmail.com)
###

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
    # session.get_results()
    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
