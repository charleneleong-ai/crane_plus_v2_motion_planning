cmake_minimum_required(VERSION 2.8.3)
project(crane_plus_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  moveit_commander
  moveit_core
  moveit_msgs
  moveit_ros_planning_interface
  moveit_visual_tools
  roscpp
  shape_msgs
  tf
  trajectory_msgs
)

find_package(Boost REQUIRED
  system
  filesystem
  date_time
  thread
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(Boost REQUIRED
  system
  filesystem
  date_time
  thread
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES crane_plus_control
   CATKIN_DEPENDS 
	actionlib 
	control_msgs 
	geometry_msgs 
	moveit_commander 
	moveit_core 
	moveit_msgs 
	moveit_ros_planning_interface 
	moveit_visual_tools 
	roscpp 
	shape_msgs 
	tf 
	trajectory_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/crane_plus_control.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/crane_plus_control_node.cpp)
add_executable(${PROJECT_NAME}_named_pose nodes/named_pose.cpp)
add_executable(${PROJECT_NAME}_parameter_tuning nodes/parameter_tuning.cpp)


## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")
set_target_properties(${PROJECT_NAME}_named_pose PROPERTIES OUTPUT_NAME named_pose PREFIX "")
set_target_properties(${PROJECT_NAME}_parameter_tuning PROPERTIES OUTPUT_NAME parameter_tuning PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_named_pose ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(${PROJECT_NAME}_parameter_tuning ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_named_pose
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_parameter_tuning
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)



