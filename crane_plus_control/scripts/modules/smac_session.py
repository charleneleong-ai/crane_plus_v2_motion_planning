#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Friday, January 25th 2019, 11:41:35 am
###

import sys
import os
import subprocess
import numpy as np

import rospkg
import rospy

from modules.session import Session

ROS_PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')+'/scripts/modules'

class SMACSession(Session):
    def __init__(self):
        super(SMACSession, self).__init__()
        rospy.loginfo('Initialising SMAC session')
        self.planner_configs_default = rospy.get_param(
            '~planner_configs_'+self.planner_select+'_default')

        

    def _create_pcs(self, planner, params_set, pcs_fp):
        try:
            pcs_file = open(pcs_fp, 'w+')
            for k, v in params_set.iteritems():
                if isinstance(v, list):
                    if(self.planner_select == "Cano_etal"):
                        param_range = ",".join(
                            [str(round(x, 2)) for x in np.arange(v[0], v[1], v[2])])
                        param_default = str(
                            self.planner_configs_default[planner][k])
                        pcs_file.write(k + ' categorical {' + param_range + '} ['+param_default+']\n')
                    elif(self.planner_select == "Burger_etal"):
                        pass
            pcs_file.close()
            rospy.loginfo('Successful writing pcs file to \n%s\n.', pcs_fp)
        except IOError:
            rospy.logerr('Error writing pcs file to \n%s\n.', pcs_fp)
            sys.exit(1)

    def _create_scenario(self, planner, scene, scenario_fp, pcs_fp):
        try:
            scenario_file = open(scenario_fp, 'w+')
            scenario_file.write('always_race_default = true\n')
            scenario_file.write('algo = python '+ROS_PKG_PATH+'/smac_run.py ' +
                                planner + ' ' + scene + ' ' + str(self.max_trials) + '\n')
            scenario_file.write('pcs_fn = ' + pcs_fp + '\n') 
            scenario_file.write('run_obj = quality\n')
            scenario_file.write('deterministic = 1\n')

            if(self.max_runtime != 'None'):
                scenario_file.write('wallclock_limit = ' + str(self.max_runtime) + '\n')
            else:
                scenario_file.write('ta_run_limit = ' + str(self.max_trials) + '\n')

            # scenario_file.write('algo-cutoff-time 40\n')
            scenario_file.close()

            rospy.loginfo('Successful writing scenario file to \n%s\n.', pcs_fp)
        except IOError:
            rospy.logerr('Error writing scenario file to \n%s\n.', scenario_fp)
            sys.exit(1)

    def run(self):

        scenario_dir =  ROS_PKG_PATH+'/SMAC3/scenarios/'
        if not os.path.exists(scenario_dir):
            os.makedirs(scenario_dir)

        for planner, params_set in self.planner_config.iteritems():
            

            pcs_fp = scenario_dir + self.planner_select + '_' + planner+'.pcs'
            scenario_fp = scenario_dir + planner + '_scenario.txt'

            self._create_pcs(planner, params_set, pcs_fp)
            self._create_scenario(planner, self.scenes[0], scenario_fp, pcs_fp)

            os.system('python3 '+ ROS_PKG_PATH +'/SMAC3/scripts/smac --scenario '+scenario_fp)

            # smac3_run = ROS_PKG_PATH +'/SMAC3/scripts/smac --scenario '+scenario_fp
            # process = subprocess.Popen(smac3_run.split(), stdout=subprocess.PIPE)
            # output, error = process.communicate() 
            # print(output)