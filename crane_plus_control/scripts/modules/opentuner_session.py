#!/usr/bin/env python
###
# File Created: Tuesday, 29th January 2019 11:29:40 am
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Tuesday, January 29th 2019, 8:47:59 pm
###
import os
import rospy

from session import Session


class OpenTunerSession(Session):
    def __init__(self):
        super(OpenTunerSession, self).__init__()
        if self.path_tune:
            rospy.loginfo('Initialising Opentuner session in %s mode from %s to %s',
                          self.mode, self.start_pose, self.target_pose)
        else:
            rospy.loginfo('Initialising Opentuner session in %s mode on full problem set', self.mode)

    def run_session(self):
        super(OpenTunerSession, self)._write_headers(self.results_path)
        args = '--no-dups '
        if(self.max_runtime != 'None'):
            args = args+'--stop-after='+str(self.max_runtime)

        for planner in self.planners:
            args = args+ ' --planner='+planner

            os.system('python '+self.ROS_PKG_PATH+'/scripts/modules/opentuner_run.py '+args)