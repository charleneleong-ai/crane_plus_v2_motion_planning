cmake_minimum_required(VERSION 2.8.3)
project(crane_plus_simulation)

find_package(catkin REQUIRED)
catkin_package()

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(PROGRAMS
  scripts/resize_gazebo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
