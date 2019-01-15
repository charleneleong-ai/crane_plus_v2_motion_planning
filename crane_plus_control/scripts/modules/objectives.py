from parameter_tuning import ParamTuningSession

# class Objective(object):
#     def __init__:
        
        

# class Objective(object):
#     def _objective(self, *args, **kwargs):
#         return weight * _compute_acceleration()

#     def _compute_acceleration(self):
#         return 0.

# class HyperOpt(ParamTuningSession):
#         # def _optimise_obj:
            


# class ElapsedTimeClass(object):
#     def _objective(self, *args, **kwargs):
#         return elapsed_time


# class MyClass(AccelerationClass, ElapsedTimeClass):
#     def _objective(self, params):
#         # Keep track of evals
#         self.eval += 1

#         # Extract param set
#         params_set = params.copy()
#         params_set.pop('planner', None)
#         params_set.pop('start_pose', None)
#         params_set.pop('target_pose', None)
#         params_set.pop('avg_run_time', None)
#         params_set.pop('avg_runs', None)
#         # Set new params
#         planner_params = moveit_msgs.msg.PlannerParams()
#         planner_params.keys = params_set.keys()
#         planner_params.values = [str(i) for i in params_set.values()]
#         self.group.set_planner_id(params['planner'])
#         self._set_planner_params(params['planner'], planner_params)
#         # print(self._get_planner_params(params['planner']))

#         avg_run_time = self._get_avg_run_time(
#             params['start_pose'], params['target_pose'], params['avg_runs'])

#         rospy.loginfo("eval: %d Avg Runtime: %f",
#                       self.eval, avg_run_time)

#         result = {'eval': self.eval, 'loss': avg_run_time, 'planner': params['planner'], 'start_pose': params['start_pose'],
#                   'target_pose': params['target_pose'], 'avg_runs': params['avg_runs']}
#         result['params'] = params_set
#         result['status'] = STATUS_OK

#         params_set.pop('type', None)
#         of_connection = open(self.results_path, 'a')
#         writer = csv.writer(of_connection)
#         writer.writerow([self.eval, avg_run_time, params['planner'], params['start_pose'],
#                          params['target_pose'], params['avg_runs'], params_set])

#         return result
