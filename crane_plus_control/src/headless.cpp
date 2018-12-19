
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
  // Specify end-effector positions in the "base_link" task frame
  arm.setPoseReferenceFrame("base_link");
  
  
  // Plan a move to the "vertical" pose
  arm.setNamedTarget("resting");
  // Execute the move
  arm.move();

  
  ros::shutdown();
  return 0;
}
