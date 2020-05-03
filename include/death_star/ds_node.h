#ifndef INCLUDE_DS_NODE_H_
#define INCLUDE_DS_NODE_H_

#include <ros/ros.h>
#include <vector>
#include <queue>
#include <map>
#include "dynamic_global_planner/graph_node.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "death_star/smartPlan.h"
#include "dynamic_global_planner/Graph.h"
#include "dynamic_global_planner/Node.h"
#include "dynamic_global_planner/Neighbour.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "std_msgs/Bool.h"


class DeathStar
{
private:

public:
	// Default constructor 
	DeathStar();

	DeathStar(ros::NodeHandle& n)
	{
		ROS_INFO("DS Created");
		nh = n;
		path_service = nh.advertiseService("gen_path", &DeathStar::PathGenerator, this);

		graph_sub = nh.subscribe < dynamic_global_planner::Graph> ("/graph_topic", 10, &DeathStar::graph_callback, this);
		path_cost_pub = nh.advertise<std_msgs::Bool>("path_cost", 1);
	}

	ros::ServiceServer path_service;

	bool PathGenerator(death_star::smartPlan::Request &req, death_star::smartPlan::Response &resp);

	float getEucledianDistance(float x1, float y1, float x2, float y2);

	void findShortestPath(float x_start, float y_start, float x_goal, float y_goal);

	Node* findNearestNode(float x, float y);
	void drawGraphonImage();
	void findPathCost(bool called_by_service);

	// Local copy of the graph
	std::vector<Node*> graph;
	std::map<std::tuple<float, float, float>, std::vector<std::tuple<float, float, float>>> graph_dict;
	std::vector<std::tuple<float, float, float>> curr_path;
	float path_cost;
	float path_cost_init;
	ros::NodeHandle nh;

	ros::Subscriber graph_sub;
	ros::Publisher path_cost_pub;
	bool subscriber_callback_executing;
	void graph_callback(const dynamic_global_planner::Graph::ConstPtr& msg);
};

#endif // INCLUDE_DS_NODE_H_
