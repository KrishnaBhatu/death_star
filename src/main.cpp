#include "ros/ros.h"
#include <iostream>
#include <string>
#include "nav_msgs/Odometry.h"
#include "death_star/ds_node.h"


int main(int argv, char **argc) {
  ros::init(argv, argc, "global_planner");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  std::string ns = ros::this_node::getNamespace();
  DeathStar dstar(nh);
  ros::spin();
  return 0; 
}

 