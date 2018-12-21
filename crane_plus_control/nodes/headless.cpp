
// Copyright 2018 Charlene Leong (charleneleong84@gmail.com)

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "headless");
  ros::NodeHandle nh;
  
  //MoveIt requires asynchronous calculation
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  //SETUP
  
  // Set up the arm planning interface
  moveit::planning_interface::MoveGroupInterface arm("arm");
  //arm.setPlannerId("BiTRRTkConfigDefault");

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  const robot_state::JointModelGroup *joint_model_group = arm.getCurrentState()->getJointModelGroup("arm");
  
  //VISUALISATION
  
  // namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  // visual_tools.deleteAllMarkers();
  
  // // Remote control is an introspection tool that allows users to step through a high level script
  // // via buttons and keyboard shortcuts in RViz
  // visual_tools.loadRemoteControl();
  
  // // RViz provides many types of markers, in this demo we will use text, cylinders, and spheres
  // Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  // text_pose.translation().z() = 1.75;
  // visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  // // Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations
  // visual_tools.trigger();
	
	
  //PLANNING TO POSE GOAL
  
  ROS_INFO_NAMED("arm", "Reference frame: %s", arm.getPlanningFrame().c_str());
  ROS_INFO_NAMED("gripper", "End effector link: %s", arm.getEndEffectorLink().c_str());
  
  //visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

  // Specify end-effector positions in the "base_link" task frame
 // arm.setPoseReferenceFrame("base_link");
 
  ROS_INFO("Moving to pose goal");
  geometry_msgs::PoseStamped pose;
  //pose.header.frame_id = "crane_plus_gripper_link";
  pose.pose.position.x = 0.3;
  pose.pose.position.y = 0.2;
  pose.pose.position.z = 0.5;
  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.707106;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 0.707106;

  arm.setPoseTarget(pose);
  moveit::planning_interface::MoveGroupInterface::Plan planned_path;
  
  bool success = (arm.plan(planned_path) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  
  ROS_INFO_NAMED("arm", "Visualizing plan 1 (pose goal) %s", success ? "SUCCESS" : "FAILED");

//Execute motion path
  if (!arm.move()) {
    ROS_WARN("Could not move to desired pose");
    return 1;
  }
 
  
  ros::shutdown();
  return 0;
}
