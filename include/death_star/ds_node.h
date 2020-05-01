#ifndef INCLUDE_DS_NODE_H_
#define INCLUDE_DS_NODE_H_

#include <ros/ros.h>
#include <vector>
#include "dynamic_global_planner/graph_node.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "death_star/smartPlan.h"

class DeathStar
{
private:

public:
	// Default constructor 
	DeathStar();

	DeathStar(ros::NodeHandle& n)
	{
		nh = n;
		path_service = nh.advertiseService("/gen_path", &DeathStar::PathGenerator, this);
	}

	ros::ServiceServer path_service;

	bool PathGenerator(death_star::smartPlan::Request &req, death_star::smartPlan::Response &resp);

	float getEucledianDistance(float x1, float y1, float x2, float y2);

	void findShortestPath(float x_start, float y_start, float x_goal, float y_goal);

	Node* findNearestNode(float x, float y);

	// Local copy of the graph
	std::vector<Node*> graph;
	std::vector<Node*> curr_path;
	ros::NodeHandle nh;
};

#endif // INCLUDE_DS_NODE_H_