#!/usr/bin/env python
###
# File Created: Tuesday, 29th January 2019 11:29:40 am
# Author: Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 11:02:12 am
###
import os
import rospy

from session import Session


class OpenTunerSession(Session):
    def __init__(self):
        super(OpenTunerSession, self).__init__()
        if self.PATH_TUNE:
            rospy.loginfo('Initialising OpenTuner session in %s mode from %s to %s\n',
                          self.MODE, self.START_POSE, self.TARGET_POSE)
        else:
            rospy.loginfo('Initialising OpenTuner session in %s mode on full problem set\n', self.MODE)

    def run_session(self):
        super(OpenTunerSession, self)._write_headers(path=self.RESULTS_PATH)
        args = '--no-dups ' # Skip duplicate param_set configs
        if(self.MAX_RUNTIME != 'None'):
            args = args+'--stop-after='+str(self.MAX_RUNTIME)   # Stop after designed runtime

        for planner in self.planners:
            args = args+' --planner='+planner
            # Need to execute OpenTuner in separate script because needs arg input to run
            os.system('python '+self.ROS_PKG_PATH+'/scripts/modules/opentuner_run.py '+args)
        
        rospy.loginfo('Saved results to %s\n', self.RESULTS_PATH)
