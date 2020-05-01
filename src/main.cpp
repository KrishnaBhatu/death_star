#include "ros/ros.h"
#include <iostream>
#include <string>
#include "nav_msgs/Odometry.h"


int main(int argv, char **argc) {
  ros::init(argv, argc, "global_planner");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();
  
}

 