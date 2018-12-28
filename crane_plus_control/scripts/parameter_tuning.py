import csv
from hyperopt import STATUS_OK
from timeit import default_timer as timer
import param_tuning_session

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

    #    # Write to the csv file ('a' means append)
    # of_connection = open(out_file, 'a')
    # writer = csv.writer(of_connection)
    # writer.writerow([loss, params, ITERATION, n_estimators, run_time])

    return {'loss': loss, 'params': params, 'iteration': ITERATION, 'train_time': run_time, 'status': STATUS_OK}

def main():
    robot, arm = init_arm()
    target = check_target()

    session = ParamTuningSession(robot, arm, planner)

    session.move_arm(target)


if __name__ == "__main__":
    main()
