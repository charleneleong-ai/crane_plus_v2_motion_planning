import csv
from hyperopt import STATUS_OK
from timeit import default_timer as timer
import rospy

planners = ['RRTConnectkConfigDefault', 'BiTRRTkConfigDefault',
            'BKPIECEkConfigDefault', 'KPIECEkConfigDefault']

#temp
planner = planners[1]

# robot, arm = init_arm()

# poses = rospy.get_param("/parameter_tuning/poses")

def objective(params, poses):

    global ITERATION
    ITERATION += 1

    start_time = timer()

    session = ParamTuningSession(robot, arm, planner)
    session.set_planner_params(params)
    session.movearm('vertical')

    for pose in poses:
        session.movearm(pose)

    run_time = timer() - start_time

    #min run_time
    loss = 1 - run_time

    return {'loss': loss, 'params': params, 'iteration': ITERATION, 'run_time': run_time, 'status': STATUS_OK}