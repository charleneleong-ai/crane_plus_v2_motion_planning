#!/usr/bin/env python

import rospy

from parameter_tuning_session import ParamTuningSession


def main():

    rospy.init_node('parameter_tuning', anonymous=True)
    # poses = rospy.get_param("/parameter_tuning/poses")
    session = ParamTuningSession()
    session.run()
    # session.get_results()


if __name__ == "__main__":
    main()
