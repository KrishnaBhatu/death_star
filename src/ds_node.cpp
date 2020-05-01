#include "death_star/ds_node.h"

DeathStar::DeathStar()
{
	ROS_INFO("Deathstar created");
    graph_sub = nh.subscribe < dynamic_global_planner::Graph> ("graph_topic", 10, &DeathStar::graph_callback, this);
}

void DeathStar::graph_callback(const dynamic_global_planner::Graph::ConstPtr& msg)
{
    ROS_INFO("Graph received");
    ROS_INFO("Nodes Size: %d", msg->mesh.size());
    ROS_INFO("Neighbours Size: %d", msg->mesh_neighbour.size());


    for(int i=0; i<msg->mesh.size(); i++)
    {
        Node* graph_node = new Node();
        graph_node->setX(msg->mesh[i].x);
        graph_node->setY(msg->mesh[i].y);
        graph_node->weight = msg->mesh[i].weight;
        for(int j=0; j<msg->mesh_neighbour[i].neighbours.size(); j++)
        {
            Node* nn_node = new Node();
            nn_node->setX(msg->mesh_neighbour[i].neighbours[j].x);
            nn_node->setY(msg->mesh_neighbour[i].neighbours[j].y);
            nn_node->weight = msg->mesh_neighbour[i].neighbours[j].weight;
            graph_node->neighbours.push_back(nn_node);
        }

        // Append this graph_node to DS_Node's graph
        graph.push_back(graph_node);
    }

    ROS_INFO("Graph Successfully Created: %d", graph.size());
}

bool DeathStar::PathGenerator(death_star::smartPlan::Request &req, death_star::smartPlan::Response &resp)
{
	findShortestPath(req.x_start, req.y_start, req.x_goal, req.y_goal);
	if(curr_path.size() < 0) return false;
	nav_msgs::Path path_planned;
	for(auto a: curr_path)
	{
		geometry_msgs::PoseStamped temp_pose;
		temp_pose.pose.position.x = a->getX();
		temp_pose.pose.position.y = a->getY();
		path_planned.poses.push_back(temp_pose);
	}
	resp.Path = path_planned;
	return true;
}

float DeathStar::getEucledianDistance(float x1, float y1, float x2, float y2)
{
    return sqrt(pow((x1-x2), 2) + pow((y1-y2), 2));
}

Node* DeathStar::findNearestNode(float x, float y)
{
    float min_val = std::numeric_limits<float>::max();
    int min_count = 0;
    for(int i = 0; i < graph.size(); i++)
    {
        float curr_dist = getEucledianDistance(x, y, graph[i]->getX(), graph[i]->getY());
        if(curr_dist < min_val)
        {
            min_val = curr_dist;
            min_count = i;
        }
    }
    return graph[min_count];
}

void DeathStar::findShortestPath(float x_start, float y_start, float x_goal, float y_goal)
{
    Node* start_node = findNearestNode(x_start, y_start);
    Node* goal_node = findNearestNode(x_goal, y_goal);
    ROS_INFO_STREAM("Start - " << start_node->getX() << ", " << start_node->getY());
    ROS_INFO_STREAM("Goal - " << goal_node->getX() << ", " << goal_node->getY());
    //Easy method will be to greedy (nearest_neighbour + Eucledian to goal)
    std::queue<Node*> visited;
    visited.push(start_node);
    curr_path.clear();
	while(visited.size() != 0)
    {
        Node* curr = visited.front();
        visited.pop();
        curr_path.push_back(curr);
        float min_h = std::numeric_limits<float>::max();
        int min_c = 0;
        if(curr == goal_node) break;
        for(int i = 0; i < curr->neighbours.size(); i++)
        {
            float curr_val  = curr->neighbours[i]->weight + getEucledianDistance(goal_node->getX(), goal_node->getY(), curr->neighbours[i]->getX(), curr->neighbours[i]->getY());
            if(curr_val < min_h)
            {
                min_h = curr_val;
                min_c = i; 
            }
        }
        visited.push(curr->neighbours[min_c]);
    }
    return;
}