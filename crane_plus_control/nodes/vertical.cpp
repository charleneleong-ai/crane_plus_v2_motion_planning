
// Copyright 2018 Charlene Leong (charleneleong84@gmail.com)


#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "resting");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Set up the arm planning interface
  moveit::planning_interface::MoveGroupInterface arm("arm");
  //arm.setPlannerId("BiTRRTkConfigDefault"); 
  //arm.setPlannerId("BKPIECEkConfigDefault");  
  //arm.setPlannerId("KPIECEkConfigDefault");  
  arm.setPlannerId("RRTstarkConfigDefault"); 

  // Specify end-effector positions in the "base_link" task frame
  arm.setPoseReferenceFrame("base_link");

  // Plan a move to the "vertical" pose
  arm.setNamedTarget("vertical");
  // Execute the move
  ROS_INFO("Moving to vertical pose");

  //Execute motion path
  if (!arm.move()) {
    ROS_WARN("Could not move to desired pose");
    return 1;
  }


  ros::shutdown();
  return 0;
}