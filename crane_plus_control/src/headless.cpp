
// Copyright 2018 Charlene Leong (charleneleong84@gmail.com)

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "headless");
  ros::NodeHandle nh;
  
  //MoveIt requires asynchronous calculation
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  // Set up the arm planning interface
  moveit::planning_interface::MoveGroupInterface arm("arm");
  //arm.setPlannerId("BiTRRTkConfigDefault");
  // Specify end-effector positions in the "base_link" task frame
  arm.setPoseReferenceFrame("base_link");
  
  // Plan a move to the "vertical" pose
   //arm.setNamedTarget("backbend");
  // Execute the move
   //arm.move();

  /*
  ROS_INFO("Moving to desired pose");
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "base_link";
  pose.pose.position.x = 0.2;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.1;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707106;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707106;
 
  
  // Plan motion path to the pose
  arm.setPoseTarget(pose);

 // Execute motion path
  if (!arm.move()) {
    ROS_WARN("Could not move to desired pose");
    return 1;
  }
  */

  
  const robot_state::JointModelGroup *joint_model_group = arm.getCurrentState()->getJointModelGroup("arm");
  
  ROS_INFO_NAMED("arm", "Reference frame: %s", arm.getPlanningFrame().c_str());
  ROS_INFO_NAMED("gripper", "End effector link: %s", arm.getEndEffectorLink().c_str());
  
  ROS_INFO("Moving to desired pose");
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 0.3;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707106;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707106;

  arm.setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan planned_path;
  
  bool success = (arm.plan(planned_path) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  
  ROS_INFO_NAMED("arm", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  arm.move();
  
  ros::shutdown();
  return 0;
}
