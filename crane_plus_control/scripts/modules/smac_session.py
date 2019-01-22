#!/usr/bin/env python
###
# File Created: Friday, January 18th 2019, 1:36:24 pm
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Tuesday, January 22nd 2019, 6:20:16 pm
###

import sys
import os
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

    def _write_pcs(self, planner, params_set, pcs_fp):
        try:
            pcs_file = open(pcs_fp, 'w')
            for k, v in params_set.iteritems():
                if isinstance(v, list):
                    if(self.planner_select == "Cano_etal"):
                        param_range = ",".join(
                            [str(round(x, 2)) for x in np.arange(v[0], v[1], v[2])])
                        param_default = str(
                            self.planner_configs_default[planner][k])
                        pcs_file.write(
                            k + ' categorical [' + param_range + '] ['+param_default+']\n')
                    elif(self.planner_select == "Burger_etal"):
                        pass
            pcs_file.close()
            rospy.loginfo('Successful writing pcs file to \n%s\n.', pcs_fp)
        except IOError:
            rospy.logerr('Error writing pcs file to \n%s\n.', pcs_fp)
            sys.exit(1)

    def _write_scenario(self, planner, scene, scenario_fp, pcs_fp):
        try:
            scenario_file = open(scenario_fp, 'w')
            scenario_file.write('use-instances = false\n')
            scenario_file.write('numberOfRunsLimit = ' +
                                str(self.max_trials) + '\n')
            scenario_file.write('runtimeLimit = ' +
                                str(self.max_runtime) + '\n')
            scenario_file.write('runObj = QUALITY\n')
            scenario_file.write('deterministic = 1\n')
            scenario_file.write('pcs-file = ' + pcs_fp + '\n')
            scenario_file.write('algo = python '+ROS_PKG_PATH+'/smac_run.py ' +
                                planner + ' ' + scene + ' ' + str(self.n_trial) + '\n')
            scenario_file.write('check-sat-consistency false \n')
            scenario_file.write('check-sat-consistency-exception false \n')
            scenario_file.write('algo-cutoff-time 40\n')
            scenario_file.write('kill-run-exceeding-captime-factor 2\n')
            # scenario_file.write('kill-run-exceeding-captime false')
            # scenario_file.write('transform-crashed-quality false')
            scenario_file.close()
            rospy.loginfo('Successful writing scenario file to \n%s\n.', pcs_fp)
        except IOError:
            rospy.logerr('Error writing scenario file to \n%s\n.', scenario_fp)
            sys.exit(1)

    def run(self):
        for planner, params_set in self.planner_config.iteritems():
            pcs_fp = ROS_PKG_PATH+'/smac/scenarios/'+planner +'/'+self.planner_select+'_'+planner+'.pcs'
            scenario_fp = ROS_PKG_PATH+'/smac/scenarios/'+planner+'/'+planner+'_scenario.txt'

            self._write_pcs(planner, params_set, pcs_fp)
            self._write_scenario(planner, self.scenes[0], scenario_fp, pcs_fp)

            os.system('cd '+ROS_PKG_PATH +'/smac && ./smac --scenario-file '+scenario_fp+' --seed 123')


# def run():
#     # for planner, params_set in planner_config.iteritems():
#     ROS_PKG_PATH = '.'
#     planner = 'RRTConnectkConfigDefault'
#     params_set = {'default': [0.1, 2.0, 0.1], 'type': 'geometric::RRTConnect'}
#     planner_select = 'Cano_etal'
#     scene = ('')


#     pcs_fp = ROS_PKG_PATH+'/smac/scenarios/'+planner + \
#         '/'+planner_select+'_'+planner+'.pcs'
#     scenario_fp = ROS_PKG_PATH+'/smac/scenarios/'+planner+'/'+planner+'_scenario.txt'

#     write_pcs(planner, params_set, pcs_fp)
#     write_scenario(planner, scene[0], scenario_fp, pcs_fp)

#     os.system('cd '+ROS_PKG_PATH +
#                 '/smac && ./smac --scenario-file '+scenario_fp+' --seed 123')

# def _write_pcs(planner, params_set, pcs_fp):
#     try:
#         pcs_file = open(pcs_fp, 'w')
#         for k, v in params_set.iteritems():
#             if isinstance(v, list):
#                 if(planner_select == "Cano_etal"):
#                     param_range = ",".join(
#                         [str(round(x, 2)) for x in np.arange(v[0], v[1], v[2])])
#                     param_default = str(
#                         planner_configs_default[planner][k])
#                     pcs_file.write(
#                         k + ' categorical [' + param_range + '] ['+param_default+']\n')
#                 elif(planner_select == "Burger_etal"):
#                     pass
#         pcs_file.close()
#         rospy.logerr('Successfully written pcs_file file. \n%s.', pcs_fp)
#     except IOError:
#         rospy.logerr('Error writing pcs file \n%s.', pcs_fp)
#         sys.exit(1)

# def _write_scenario(planner, scene, scenario_fp, pcs_fp):
#     try:
#         scenario_file = open(scenario_fp, 'w')
#         scenario_file.write('use-instances = false\n')
#         scenario_file.write('numberOfRunsLimit = ' +
#                             str(max_trials) + '\n')
#         scenario_file.write('runtimeLimit = ' +
#                             str(max_runtime) + '\n')
#         scenario_file.write('runObj = QUALITY\n')
#         scenario_file.write('deterministic = 1\n')
#         scenario_file.write('pcs-file = ' + pcs_fp + '\n')
#         scenario_file.write('algo = python smac_run.py ' +
#                             planner + ' ' + scene + ' ' + n_trial + '\n')
#         scenario_file.write('check-sat-consistency false \n')
#         scenario_file.write('check-sat-consistency-exception false \n')
#         scenario_file.write('algo-cutoff-time 40\n')
#         scenario_file.write('kill-run-exceeding-captime-factor 2\n')
#         # scenario_file.write('kill-run-exceeding-captime false')
#         # scenario_file.write('transform-crashed-quality false')
#         scenario_file.close()
#         rospy.logerr(
#             'Successfully written scenario file. \n%s', scenario_fp)
#     except IOError:
#         rospy.logerr('Error writing scenario file. \n%s', scenario_fp)
#         sys.exit(1)

# scene = ["narrow_passage"]

# if __name__ == "__main__":
#     rospy.init_node('smac_session', anonymous=True)
#     run()
