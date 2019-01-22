#!/usr/bin/env python
###
# File Created: Monday, January 21st 2019, 10:55:57 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Last Modified: Tuesday, January 22nd 2019, 1:59:08 pm
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

logging.basicConfig(level=logging.INFO)
from session import Session

class SMACRun(Session):
    def __init__(self);
        super(SMACRun, self).__init__()


if __name__ == "__main__":
    session = 