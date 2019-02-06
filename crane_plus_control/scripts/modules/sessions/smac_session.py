#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author: Charlene Leong charleneleong84@gmail.com
# Modified By: Charlene Leong
# Last Modified: Wednesday, February 6th 2019, 4:47:11 pm
###

import sys
import os
import datetime
import numpy as np

import rospy

from tuning_session import TuningSession


class SMACSession(TuningSession):
    """
    SMAC Session
    _create_pcs(planner, params_set, pcs_fp): Creates pcs file
    _create_scenario(planner, scene, scenario_fp, pcs_fp): Creates the scenario file
    run(): Executes SMAC on the smac_run.py with pcs and scenario file
    """
    
    def __init__(self):
        super(SMACSession, self).__init__()
        if self.PATH_TUNE:
            rospy.loginfo('Initialising SMAC session in %s mode from %s to %s\n',
                          self.MODE, self.START_POSE, self.TARGET_POSE)
        else:
            rospy.loginfo('Initialising SMAC session in %s mode on full problem set\n', self.MODE)

        self.planner_configs_default = rospy.get_param('~planner_configs_'+self.PLANNER_SELECT+'_default')

        self.SMAC_PATH = self.ROS_PKG_PATH+'/scripts/modules'


    def _create_pcs(self, planner, params_set, pcs_fp):
        """Creates Parameter Configuration Space (PCS) file"""
        # https://automl.github.io/SMAC3/master/options.html?highlight=algo#parameter-configuration-space-pcs
        try:
            pcs_file = open(pcs_fp, 'w+')
            for k, v in params_set.iteritems():
                if isinstance(v, list):
                    param_default = str(self.planner_configs_default[planner][k])
                    if(self.PLANNER_SELECT == "Cano_etal"):
                        # <name> categorical {array} [default]
                        param_range = ",".join([str(round(x, 2)) for x in np.arange(v[0], v[1], v[2])])
                        pcs_file.write(k+' categorical {'+param_range+'} ['+param_default+']\n')
                    elif(self.PLANNER_SELECT == "Burger_etal"):
                        # <name> real [min, max] [default]
                        pcs_file.write(k+' real ['+str(v[0])+', '+ str(v[1]) +'] ['+param_default+']\n')
            pcs_file.close()
            rospy.loginfo('Successful writing pcs file to \n%s\n.', pcs_fp)
        except IOError:
            rospy.logerr('Error writing pcs file to \n%s\n.', pcs_fp)
            sys.exit(1)

    def _create_scenario(self, planner, scene, scenario_fp, pcs_fp):
        """Creates scenario file"""
        # https://automl.github.io/SMAC3/master/options.html#scenario
        try:
            scenario_file = open(scenario_fp, 'w+')
            # Race new incumbents always against default configuration
            scenario_file.write('always_race_default = true\n')
            # Program to run and parameters
            scenario_file.write('algo = python '+self.SMAC_PATH+'/sessions/smac_run.py ' +
                                planner+' '+scene+' '+str(self.MAX_TRIALS)+'\n')
            # Specifies the path to the PCS-file.
            scenario_file.write('pcs_fn = '+pcs_fp+'\n') 
            # Defines what metric to optimise - quality(loss) or runtime
            scenario_file.write('run_obj = quality\n')
            # If true, the optimisation process will be repeatable
            scenario_file.write('deterministic = 1\n')
            if(self.MAX_RUNTIME != 'None'):
                # Maximum amount of wallclock-time used for optimization.  Default: inf.
                scenario_file.write('wallclock_limit = '+str(self.MAX_RUNTIME)+'\n')
            else:
                # Maximum number of algorithm-calls during optimisation.  Default: inf.
                scenario_file.write('ta_run_limit = '+str(self.MAX_TRIALS)+'\n')
            
            currentDT = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            scenario_file.write('output_dir = '+self.SMAC_PATH+'/SMAC3/output/'+ \
                            self.PLANNER_SELECT+'_'+planner+'_'+currentDT+'\n')
            scenario_file.close()

            rospy.loginfo('Successful writing scenario file to \n%s\n.', scenario_fp)
        except IOError:
            rospy.logerr('Error writing scenario file to \n%s\n.', scenario_fp)
            sys.exit(1)

    def _load_search_space(self, params_set, *args, **kwargs):
        planner = args[0]
        pcs_fp = args[1]
        scenario_fp = args[2]
        self._create_pcs(planner, params_set, pcs_fp)
        self._create_scenario(planner, self.scenes[0], scenario_fp, pcs_fp)

    def run_session(self):
        self._write_headers(path=self.RESULTS_PATH)

        SCENARIO_DIR =  self.SMAC_PATH+'/SMAC3/scenarios/'
        if not os.path.exists(SCENARIO_DIR):
            os.makedirs(SCENARIO_DIR)

        for planner in self.planners:
            params_set = dict(self.planner_config[planner].items())
            pcs_fp = SCENARIO_DIR+self.PLANNER_SELECT+'_'+planner+'.pcs'
            scenario_fp = SCENARIO_DIR+planner+'_scenario.txt'

            # Write the scenario and pcs files to define SMAC run
            self._load_search_space(params_set, planner, pcs_fp, scenario_fp)

            # Need to execute SMAC run in separate script bcz py2 and py3 incompatibility
            # python3 smac --scenario SCENARIO --seed INT --verbose_level LEVEL --mode MODE
            os.system('python3 '+self.SMAC_PATH+'/SMAC3/scripts/smac --scenario '+scenario_fp)

        rospy.loginfo('Saved results to %s\n', self.RESULTS_PATH)
