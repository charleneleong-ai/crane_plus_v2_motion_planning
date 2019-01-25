#!/usr/bin/env python
###
# File Created: Wednesday, 23rd January 2019 11:41:51 am
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Thursday, January 24th 2019, 4:00:20 pm
###

import sys
import os
import rospy


if __name__ == "__main__":

    rospy.init_node('resize_gazebo', anonymous=True)
    gazebo_ini_fp = os.path.join(os.environ['HOME'], '.gazebo/gui.ini')

    try:
        gazebo_ini_file = open(gazebo_ini_fp, 'w+')
        gazebo_ini_file.write('[geometry]\n')
        gazebo_ini_file.write('x=65\n')
        gazebo_ini_file.write('y=415\n')
        gazebo_ini_file.write('width=1000\n')
        gazebo_ini_file.write('height=570\n')
        gazebo_ini_file.close()
    except IOError:
        rospy.logerr('Error writing gazebo ini file to \n%s\n.', gazebo_ini_fp)
        sys.exit(1)
