#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Tuesday, January 29th 2019, 5:08:09 pm
###

import sys
import os
import numpy as np

import rospkg
import rospy

from session import Session


class SMACSession(Session):
    """
    SMAC Session
    _create_pcs(planner, params_set, pcs_fp): Creates pcs file
    _create_scenario(planner, scene, scenario_fp, pcs_fp): Creates the scenario file
    run(): Executes SMAC on the smac_run.py with pcs and scenario file
    """
    
    def __init__(self):
        super(SMACSession, self).__init__()
        if self.path_tune:
            rospy.loginfo('Initialising SMAC session in %s mode from %s to %s',
                          self.mode, self.start_pose, self.target_pose)
        else:
            rospy.loginfo('Initialising SMAC session in %s mode on full problem set', self.mode)

        self.planner_configs_default = rospy.get_param(
            '~planner_configs_'+self.planner_select+'_default')

        self.SMAC_PATH = self.ROS_PKG_PATH+'/scripts/modules'


    def _create_pcs(self, planner, params_set, pcs_fp):
        try:
            pcs_file = open(pcs_fp, 'w+')
            for k, v in params_set.iteritems():
                if isinstance(v, list):
                    param_default = str(self.planner_configs_default[planner][k])
                    if(self.planner_select == "Cano_etal"):
                        param_range = ",".join([str(round(x, 2)) for x in np.arange(v[0], v[1], v[2])])
                        pcs_file.write(k + ' categorical {' + param_range + '} ['+param_default+']\n')
                    elif(self.planner_select == "Burger_etal"):
                        pcs_file.write(k + ' real [' + str(v[0])+', '+ str(v[1]) +'] ['+param_default+']\n')
            pcs_file.close()
            rospy.loginfo('Successful writing pcs file to \n%s\n.', pcs_fp)
        except IOError:
            rospy.logerr('Error writing pcs file to \n%s\n.', pcs_fp)
            sys.exit(1)

    def _create_scenario(self, planner, scene, scenario_fp, pcs_fp):
        try:
            scenario_file = open(scenario_fp, 'w+')
            scenario_file.write('always_race_default = true\n')
            scenario_file.write('algo = python '+self.SMAC_PATH+'/smac_run.py ' +
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

    def _load_search_space(self, planner, params_set, *args, **kwargs):
        pcs_fp = args[0]
        scenario_fp = args[1]
        self._create_pcs(planner, params_set, pcs_fp)
        self._create_scenario(planner, self.scenes[0], scenario_fp, pcs_fp)

    def run_session(self):
    
        super(SMACSession, self)._write_headers(self.results_path)

        scenario_dir =  self.SMAC_PATH+'/SMAC3/scenarios/'
        if not os.path.exists(scenario_dir):
            os.makedirs(scenario_dir)

        for planner, params_set in self.planner_config.iteritems():
            pcs_fp = scenario_dir + self.planner_select + '_' + planner+'.pcs'
            scenario_fp = scenario_dir + planner + '_scenario.txt'

            self._load_search_space(planner, params_set, pcs_fp, scenario_fp)
        
            os.system('python3 '+ self.SMAC_PATH +'/SMAC3/scripts/smac --scenario '+scenario_fp)

        print('\n')
        rospy.loginfo('Saved results to %s', self.results_path)
        print('\n')