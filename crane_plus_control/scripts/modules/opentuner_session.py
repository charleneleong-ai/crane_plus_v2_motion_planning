#!/usr/bin/env python
###
# File Created: Tuesday, 29th January 2019 11:29:40 am
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 30th 2019, 10:31:13 am
###
import os
import rospy

from session import Session


class OpenTunerSession(Session):
    def __init__(self):
        super(OpenTunerSession, self).__init__()
        if self.path_tune:
            rospy.loginfo('Initialising OpenTuner session in %s mode from %s to %s',
                          self.mode, self.start_pose, self.target_pose)
        else:
            rospy.loginfo('Initialising OpenTuner session in %s mode on full problem set', self.mode)

    def run_session(self):
        super(OpenTunerSession, self)._write_headers(self.results_path)
        args = '--no-dups ' # Skip duplicate param_set configs
        if(self.max_runtime != 'None'):
            args = args+'--stop-after='+str(self.max_runtime)   # Stop after designed runtime

        for planner in self.planners:
            args = args+ ' --planner='+planner
            # Need to execute OpenTuner in separate script because needs arg input to run
            os.system('python '+self.ROS_PKG_PATH+'/scripts/modules/opentuner_run.py '+args)
        
        print('\n')
        rospy.loginfo('Saved results to %s\n', self.results_path)
