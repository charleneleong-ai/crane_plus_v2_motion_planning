#!/usr/bin/env python

import sys
import rospy
import moveit_commander
from parameter_tuning_session import ParamTuningSession


def main():

    # parser = argparse.ArgumentParser()
    # parser.add_argument('--max_trials', type=int, default=30,
    #                     help='Max num of trials for hyperopt')
    # args = parser.parse_args()
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('parameter_tuning', anonymous=True)
    # poses = rospy.get_param("/parameter_tuning/poses")
    session = ParamTuningSession()
    session.run()
    # session.get_results()

    moveit_commander.roscpp_shutdown()

if __name__ == "__main__":
    main()
