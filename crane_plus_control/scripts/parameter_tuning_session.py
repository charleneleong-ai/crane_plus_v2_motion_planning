import sys
from timeit import default_timer as timer
from collections import OrderedDict

import csv
import pprint
import pandas as pd
import numpy as np

import rospkg
import rospy

import moveit_commander
from moveit_msgs.srv import GetPlannerParams, SetPlannerParams
import moveit_msgs.msg

from hyperopt import hp, rand, tpe, Trials, fmin, STATUS_OK
from hyperopt.pyll.stochastic import sample

from objectives import objectives


class ParamTuningSession(object):
    """
    Class for a Parameter Tuning Session
    """
    PKG_PATH = rospkg.RosPack().get_path('crane_plus_control')
    PLANNING_TIME = 2  # seconds

    def __init__(self):

        planner_select, start_pose, target_pose, mode = self._load_params()
        self.mode = mode
        self.name = planner_select+"_"+mode
        self.n_trial = 0
        self.max_trials = rospy.get_param("/parameter_tuning/max_trials")
        self.start_pose = start_pose
        self.target_pose = target_pose

        if self.mode is "baseline":
            self.planner_config = rospy.get_param(
                "/parameter_tuning/planner_configs_"+planner_select+"_default")
        else:
            self.planner_config = rospy.get_param(
                "/parameter_tuning/planner_configs_"+planner_select+"_tune")
        assert isinstance(self.planner_config, dict)

        self.planners = self.planner_config.keys()
        self.results_path = self.PKG_PATH+'/results/'+self.name+".csv"

        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")
        # self.planning_frame = self.group.get_planning_frame()  # "/world"
        # self.scene = moveit_commander.PlanningSceneInterface()

    def _load_params(self):
        planner_select = rospy.get_param(
            "/parameter_tuning/planner_config")
        if planner_select not in ['Cano_etal']:
                rospy.logerr("Invalid planner config select")
                sys.exit(1)

        start_pose = rospy.get_param("/parameter_tuning/start_pose")
        target_pose = rospy.get_param("/parameter_tuning/target_pose")
        named_states = rospy.get_param("/parameter_tuning/named_states")
        if target_pose not in named_states:
            rospy.logerr('target_pose not in list of named_states')
            rospy.logerr(named_states)
            sys.exit(1)
        elif start_pose not in named_states:
            rospy.logerr('start_pose not in list of named_states')
            rospy.logerr(named_states)
            sys.exit(1)

        mode = rospy.get_param("/parameter_tuning/mode")
        if mode not in ['baseline', 'tpe', 'rand']:
            rospy.logerr("Invalid mode.")
            sys.exit(1)

        return planner_select, start_pose, target_pose, mode

    def _get_planner_params(self, planner_id):
        # rospy.loginfo('Waiting for get_planner_params')
        rospy.wait_for_service('get_planner_params')
        get_planner_params = rospy.ServiceProxy(
            'get_planner_params', GetPlannerParams)
        try:
            req = get_planner_params(planner_id, "arm")
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)
        return req.params

    def _set_planner_params(self, planner_id, params):
        # rospy.loginfo('Waiting for set_planner_params')
        rospy.wait_for_service('set_planner_params')
        set_planner_params = rospy.ServiceProxy(
            'set_planner_params', SetPlannerParams)
        try:
            set_planner_params(planner_id, "arm", params, True)
            rospy.loginfo('Parameters updated')
        except rospy.ServiceException as e:
            rospy.logerr('Failed to get params: %s', e)

    def _move_arm(self, pose):
        # Setting goal
        self.group.set_named_target(pose)
        plan = self.group.plan()

        # Visualising trajectory
        display_trajectory_publisher = rospy.Publisher('/group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=20)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)

        #rospy.loginfo("Moving to %s pose", pose)
        plan = self.group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        self.group.stop()

    def _get_avg_run_time(self, start_pose, target_pose, iter):
        run_times = []
        for _ in range(iter):
            self._move_arm(start_pose)
            start_time = timer()
            self._move_arm(target_pose)
            run_time = timer() - start_time
            run_times.append(run_time)

        return sum(run_times)/float(len(run_times))

    def _obtain_baseline(self, start_pose, target_pose, iter):
        # for OMPL defaults
        # self.planner_config = rospy.get_param('/group/planner_configs/')
        # self.planner_config = dict((k, self.planner_config[k]) for k in self.planners if k in self.planner_config)

        avg_run_times = []
        for p in self.planners:
            rospy.loginfo(
                "Executing %s baseline: Averaging over %d runs", p, iter)
            self.group.set_planner_id(p)

            avg_run_time = self._get_avg_run_time(
                start_pose, target_pose, iter)
            avg_run_times.append(avg_run_time)

            self.planner_config[p].pop('type', None)

        result = OrderedDict([('avg_run_time', avg_run_time), ('planner', p), ('start_pose', start_pose),
                              ('target_pose', target_pose), ('avg_runs', iter), ('params', self.planner_config.values())])

        df = pd.DataFrame(result)
        df.to_csv(self.results_path, index=False)

        # Converting nested dict to pd DataFrame
        # ids = []
        # frames = []
        # for id, d in sorted(self.planner_config.iteritems(), key=lambda (k, v): (v, k)):
        #     ids.append(id)
        #     frames.append(pd.DataFrame.from_dict(d, orient='index'))
        # df = pd.concat(frames, keys=ids)

        avg_run_times = pd.DataFrame(
            {'planners': self.planners, 'run_times (s)': avg_run_times})
        print("\n")
        print(avg_run_times)

        return df, avg_run_times

    def _objective(self, params):
        # raise NotImplementedError, "Should be implemented in child class"
        # Keep track of trials
        self.n_trial += 1

        # Extract param set
        params_set = params.copy()
        params_set.pop('planner', None)
        params_set.pop('start_pose', None)
        params_set.pop('target_pose', None)
        params_set.pop('avg_run_time', None)
        params_set.pop('avg_runs', None)
        # Set new params
        planner_params = moveit_msgs.msg.PlannerParams()
        planner_params.keys = params_set.keys()
        planner_params.values = [str(i) for i in params_set.values()]
        self.group.set_planner_id(params['planner'])
        self._set_planner_params(params['planner'], planner_params)
        # print(self._get_planner_params(params['planner']))

        avg_run_time = self._get_avg_run_time(
            params['start_pose'], params['target_pose'], params['avg_runs'])

        accel = self.get_accel()

        rospy.loginfo("n_trial: %d Avg Runtime: %f",
                      self.n_trial, avg_run_time)

        result = {'n_trial': self.n_trial, 'loss': avg_run_time, 'planner': params['planner'], 'start_pose': params['start_pose'],
                  'target_pose': params['target_pose'], 'avg_runs': params['avg_runs']}
        result['params'] = params_set
        result['status'] = STATUS_OK

        params_set.pop('type', None)
        of_connection = open(self.results_path, 'a')
        writer = csv.writer(of_connection)
        writer.writerow([self.n_trial, avg_run_time, params['planner'], params['start_pose'],
                         params['target_pose'], params['avg_runs'], params_set])

        return result

    def _optimise_obj(self, start_pose, target_pose, iter):
        # File to save first results
        of_connection = open(self.results_path, 'w')
        writer = csv.writer(of_connection)
        writer.writerow(['n_trial', 'avg_run_time', 'planner',
                         'start_pose', 'target_pose', 'avg_runs', 'params'])
        of_connection.close()

        # Setting up the parameter search space and parameters
        for planner, params in self.planner_config.iteritems():
            params = dict(self.planner_config[planner].items())
            # pprint.pprint(param_grid)

            for k, v in params.iteritems():
                if isinstance(v, list):
                    begin_range = v[0]
                    end_range = v[1]
                    step = v[2]
                    # Discrete uniform dist
                    params[k] = hp.quniform(k, begin_range, end_range, step)

            params['planner'] = planner
            params['start_pose'] = start_pose
            params['target_pose'] = target_pose
            params['avg_runs'] = iter
            # pprint.pprint(params)

            print("\n")
            rospy.loginfo("Executing %s on %s:  Max trials: %d Averaging over %d runs", self.mode,
                          params['planner'], self.max_trials, params['avg_runs'])
            # Reset to num_trials to zero for each planner
            self.n_trial = 0
            #Run optimisation

            if self.mode == "tpe":
                algo = tpe.suggest
            elif self.mode == "rand":
                algo = rand.suggest
            best = fmin(fn=self._objective, space=params, algo=algo,
                        max_evals=self.max_trials, trials=Trials())
            # pprint.pprint(best)

    def run(self):
        iter = 1
        if self.mode is "baseline":
            self._obtain_baseline(self.start_pose, self.target_pose, iter)
        else:
            self._optimise_obj(self.start_pose, self.target_pose, iter)

    def get_results(self):
        # try:
        # catch:

        results = pd.read_csv(self.results_path)
        # Sort with best scores on top and reset index for slicing

        for p in results.planner.unique():
            planner_df = results[results['planner'] == p]
            #print(planner_df)
            planner_df.sort_values(
                'avg_run_time', ascending=True, inplace=True)
            planner_df.reset_index(inplace=True, drop=True)
            print(planner_df.head())
