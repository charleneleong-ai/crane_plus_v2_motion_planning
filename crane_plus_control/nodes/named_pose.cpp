
// Copyright 2018 Charlene Leong (charleneleong84@gmail.com)

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
  std::string named_pose;

  //getNamedTargets
  //https://docs.ros.org/api/moveit_ros_planning_interface/html/classmoveit_1_1planning__interface_1_1MoveGroupInterface.html#a8fdc429c39a619d9935d9d15efbed87d

  if (argc == 1 || argc > 2)
  {
    printf("rosrun crane_plus_control named_pose [<pose>] \n");
    return 1;
  }
  else if (argc == 2)
  {
    
    if (strcmp(argv[1], "vertical") == 0)
    {
      named_pose = "vertical";
    }
    else if (strcmp(argv[1], "backbend") == 0)
    {
      named_pose = "backbend";
    }
    else if (strcmp(argv[1], "resting") == 0)
    {
      named_pose = "resting";
    }else{
      printf("Invalid pose \n");
      return 1;
    }
  }

  ros::init(argc, argv, "named_pose");
  ros::NodeHandle nh;

  //MoveIt requires asynchronous calculation
  ros::AsyncSpinner spinner(2);
  spinner.start();

  //SETUP

  // Set up the arm planning interface
  moveit::planning_interface::MoveGroupInterface arm("arm");

  arm.setPlannerId("RRTConnectkConfigDefault");

  arm.setPoseReferenceFrame("base_link");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  const robot_state::JointModelGroup *joint_model_group = arm.getCurrentState()->getJointModelGroup("arm");

  //PLANNING TO NAMED GOAL

  // Plan a move to the named pose
  arm.setNamedTarget(named_pose);
  // Execute the move
  ROS_INFO("Moving to %s pose", named_pose.c_str());

  moveit::planning_interface::MoveGroupInterface::Plan planned_path;

  bool success = (arm.plan(planned_path) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("arm", "Visualizing plan %s", success ? "SUCCESS" : "FAILED");

  //Execute motion path
  if (!arm.move())
  {
    ROS_WARN("Could not move to %s pose", named_pose.c_str());
    return 1;
  }

  ros::shutdown();
  return 0;
}
