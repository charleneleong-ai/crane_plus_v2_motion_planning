#!/usr/bin/env python
###
# File Created: Wednesday, 23rd January 2019 11:41:51 am
# Author:  Charlene Leong (charleneleong84@gmail.com>)
# Modified By: Charlene Leong
# Last Modified: Wednesday, January 23rd 2019, 4:49:19 pm
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
        gazebo_ini_file.write('x=55\n')
        gazebo_ini_file.write('y=1082\n')
        gazebo_ini_file.write('width=1383\n')
        gazebo_ini_file.write('height=1040\n')
        gazebo_ini_file.close()
    except IOError:
        rospy.logerr('Error writing gazebo ini file to \n%s\n.', gazebo_ini_fp)
        sys.exit(1)
