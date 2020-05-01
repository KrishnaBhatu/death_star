#ifndef INCLUDE_DS_NODE_H_
#define INCLUDE_DS_NODE_H_

#include <ros/ros.h>
#include <vector>
#include "dynamic_global_planner/graph_node.h"
class DeathStar
{
private:

public:
	// Default constructor 
	DeathStar();

	// Local copy of the graph
	std::vector<Node*> graph;
};

#endif // INCLUDE_DS_NODE_H_