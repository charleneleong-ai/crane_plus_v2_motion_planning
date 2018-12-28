import csv
from hyperopt import STATUS_OK
from timeit import default_timer as timer

planners = ['RRTConnectkConfigDefault', 'BiTRRTkConfigDefault',
            'BKPIECEkConfigDefault', 'KPIECEkConfigDefault']

#temp
planner = planners[1]

robot, arm = init_arm()
target = check_target()

def objective(params):

    global ITERATION
    ITERATION += 1

    session = ParamTuningSession(robot, arm, planner, start, target)

    return {'loss': loss, 'params': params, 'iteration': ITERATION, 'train_time': run_time, 'status': STATUS_OK}