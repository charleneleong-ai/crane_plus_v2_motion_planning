/*
 * File Created: Saturday, January 12th 2019, 11:23:55 am
 * Author: Charlene Leong charleneleong84@gmail.com
 * Last Modified: Sunday, January 20th 2019, 12:15:25 am
 * Modified By: Charlene Leong
 */

#include <iostream>
#include <unordered_map>

#include <ros/ros.h>
#include <ros/param.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include "yaml-cpp/yaml.h"

template<typename K, typename V>
void print_map(std::map<K,V> const &m){
    for (auto const& param: m) {
        std::cout << "{" << param.first << ": " << param.second << "}\n";
    }
}



int main(int argc, char **argv) {

  //   if (argc == 1 || argc > 2)
  // {
  //   printf("filepath\n");
  //   return 1;
  // }
  // }

  ros::init(argc, argv, "parameter_tuning");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Set up the arm planning interface
  moveit::planning_interface::MoveGroupInterface arm("arm");
  // Specify end-effector positions in the "base_link" task frame
  arm.setPoseReferenceFrame("base_link");
  //arm.setPlannerId("BiTRRTkConfigDefault"); 
  //arm.setPlannerId("BKPIECEkConfigDefault");  
  //arm.setPlannerId("KPIECEkConfigDefault");  
  arm.setPlannerId("RRTstarkConfigDefault"); 

  std::string plannerID = arm.getPlannerId();
  std::map< std::string, std::string >	params = arm.getPlannerParams(plannerID,"arm");
  
  printf("%s \n", plannerID.c_str());
  print_map(params);

  // if(!nh.getParam("", cols))
  //   {
  //       ROS_ERROR_STREAM("Translation vector (cols) could not be read.");
  //       return 0;
  //   }

  std::string file_path = "/home/c/crane/config/parameter_tuning.yaml";

  // YAML::Node baseNode = YAML::LoadFile(params);



  // Plan a move to the "resting" pose
  arm.setNamedTarget("resting");
  // Execute the move
  ROS_INFO("Moving to resting pose");

  //Execute motion path
  if (!arm.move()) {
    ROS_WARN("Could not move to desired pose");
    return 1;
  }


  // ros::shutdown();
  // return 0;
}