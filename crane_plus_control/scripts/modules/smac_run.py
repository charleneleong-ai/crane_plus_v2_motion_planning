#!/usr/bin/env python
###
# File Created: Monday, January 21st 2019, 10:55:57 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Last Modified: Tuesday, January 22nd 2019, 7:16:27 pm
# Modified By: Charlene Leong
###

import sys
import os
import time
import logging

import signal

import numpy
#import cPickle as pickle

import rospy
import moveit_commander
import moveit_msgs.msg
import moveit_msgs.srv
import geometry_msgs
import shape_msgs
import std_msgs
from datetime import datetime as dt
import sys, math, random

logging.basicConfig(level=logging.INFO)
from modules.session import Session

class SMACRun(Session):
    def __init__(self):
        super(SMACRun, self).__init__()


if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)


# For black box function optimization, we can ignore the first 5 arguments. 
# The remaining arguments specify parameters using this format: -name value 

    x1 = 0 
    x2 = 0

    for i in range(len(sys.argv)-1):  
        if (sys.argv[i] == '-x1'):
            x1 = float(sys.argv[i+1])
        elif(sys.argv[i] == '-x2'):
            x2 = float(sys.argv[i+1])   
    
    # Compute the branin function:
    yValue = (x2 - (5.1 / (4 * math.pi * math.pi)) *x1*x1 + (5 / (math.pi)) *x1 -6) ** 2 + 10*(1- (1 / (8 * math.pi))) * math.cos(x1) + 10




    # SMAC has a few different output fields; here, we only need the 4th output:
    print "Result of algorithm run: SUCCESS, 0, 0, %f, 0" % yValue
    
