#include "death_star/ds_node.h"

struct Compare
{
    bool operator()(const std::tuple<float, float, float>& a, const std::tuple<float, float, float>& b) const
    {
        return std::get<2>(a) > std::get<2>(b);
    }
};

DeathStar::DeathStar()
{
	ROS_INFO("Deathstar created");
    
}

void DeathStar::graph_callback(const dynamic_global_planner::Graph::ConstPtr& msg)
{
    subscriber_callback_executing = true;
    ROS_INFO("Graph received");
    ROS_INFO_STREAM("Nodes Size: " << msg->mesh.size());
    ROS_INFO_STREAM("Neighbours Size: " <<msg->mesh_neighbour.size());

    graph.clear();
    graph_dict.clear();
    for(int i=0; i<msg->mesh.size(); i++)
    {
        
        Node* graph_node = new Node();
        graph_node->setX(msg->mesh[i].x);
        graph_node->setY(msg->mesh[i].y);
        graph_node->weight = msg->mesh[i].weight;
        std::vector<std::tuple<float, float, float>> neigh;
        for(int j=0; j<msg->mesh_neighbour[i].neighbours.size(); j++)
        {
            Node* nn_node = new Node();
            nn_node->setX(msg->mesh_neighbour[i].neighbours[j].x);
            nn_node->setY(msg->mesh_neighbour[i].neighbours[j].y);
            nn_node->weight = msg->mesh_neighbour[i].neighbours[j].weight;
            graph_node->neighbours.push_back(nn_node);
            
            std::tuple<float, float, float> temp_tup(nn_node->getX(), nn_node->getY(), nn_node->weight);
            neigh.push_back(temp_tup);
        }
        std::tuple<float, float, float> temp_gr(graph_node->getX(), graph_node->getY(), graph_node->weight);
        graph_dict.insert({temp_gr, neigh});
        // Append this graph_node to DS_Node's graph
        graph.push_back(graph_node);
    }

    ROS_INFO_STREAM("Graph Successfully Created: " << graph.size());
    subscriber_callback_executing = false;
}

bool DeathStar::PathGenerator(death_star::smartPlan::Request &req, death_star::smartPlan::Response &resp)
{
    ROS_INFO_STREAM("Is subscriber on --->" << subscriber_callback_executing);
	findShortestPath(req.x_start, req.y_start, req.x_goal, req.y_goal);
	ROS_INFO_STREAM("Current Path Length: - " << curr_path.size());
	//for(auto a: curr_path) ROS_INFO_STREAM(a->getX() << ", " << a->getY());
	if(curr_path.size() < 0) return false;
	nav_msgs::Path path_planned;
	for(auto a: curr_path)
	{
		geometry_msgs::PoseStamped temp_pose;
		temp_pose.pose.position.x = std::get<0>(a);
		temp_pose.pose.position.y = std::get<1>(a);
		path_planned.poses.push_back(temp_pose);
	}
	drawGraphonImage();
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
    while(subscriber_callback_executing){}
    std::map<std::tuple<float, float>, float> old_weight;
    std::map<std::tuple<float, float>, float> curr_g_cost;
	
    if(graph.size() <= 0) return;
    Node* start_node = findNearestNode(x_start, y_start);
    Node* goal_node = findNearestNode(x_goal, y_goal);
    ROS_INFO_STREAM("Start - " << start_node->getX() << ", " << start_node->getY());
    ROS_INFO_STREAM("Goal - " << goal_node->getX() << ", " << goal_node->getY());
    //Easy method will be to greedy (nearest_neighbour + Eucledian to goal)
    std::priority_queue<std::tuple<float, float, float>, std::vector<std::tuple<float, float, float>>, Compare> visited_pq;
    //std::queue<std::tuple<float, float, float>> visited;
    std::set<std::tuple<float, float, float>> already_gone;
    std::map<std::tuple<float, float, float>, std::tuple<float, float, float>> child_parent;
    //std::set<std::tuple<float, float>> already_gone_no_weight;
    std::tuple<float, float, float> start_tup(start_node->getX(), start_node->getY(), start_node->weight);
    std::tuple<float, float, float> goal_tup(goal_node->getX(), goal_node->getY(), goal_node->weight);
    std::tuple<float, float> temp_check(std::get<0>(start_tup), std::get<1>(start_tup));
    curr_g_cost.insert({temp_check, std::get<2>(start_tup)});
    visited_pq.push(start_tup);
    curr_path.clear();

	while(visited_pq.size() != 0)
    {
        std::tuple<float, float, float> curr = visited_pq.top();
        visited_pq.pop();
        std::tuple<float, float> temp_check(std::get<0>(curr), std::get<1>(curr));
        if(old_weight.find(temp_check) != old_weight.end()) std::get<2>(curr) = old_weight[temp_check];
        
        already_gone.insert(curr);
		ROS_INFO_STREAM(std::get<0>(curr) << ", " << std::get<1>(curr));
        float min_h = std::numeric_limits<float>::max();
        int min_c = 0;
        if(std::get<0>(curr) == std::get<0>(goal_tup) && std::get<1>(curr) == std::get<1>(goal_tup))
        {
            curr_path.push_back(curr);
            while(1)
            {
                std::tuple<float, float, float> curr_tup = child_parent[curr];            
                curr_path.push_back(curr_tup);
                curr = curr_tup;
                if(std::get<0>(curr) == std::get<0>(start_tup) && std::get<1>(curr) == std::get<1>(start_tup))
                {
                    std::reverse(curr_path.begin(), curr_path.end());
                    break;
                }
            }
            break;
        }
		ROS_INFO_STREAM("Neighbour size " << graph_dict[curr].size());
        for(int i = 0; i < graph_dict[curr].size(); i++)
        {
            if(child_parent.find(graph_dict[curr][i]) == child_parent.end())
            {
                child_parent.insert({graph_dict[curr][i], curr});
            }
            else
            {
                //Check for lowest g cost or change parent
                std::tuple<float, float> old_par(std::get<0>(graph_dict[curr][i]), std::get<1>(graph_dict[curr][i]));
                float old_g_cost = curr_g_cost[old_par];
                float curr_g_cost_f = curr_g_cost[temp_check];
                if(curr_g_cost_f < old_g_cost)
                {
                    //Change parent
                    child_parent[graph_dict[curr][i]] = curr; 
                }
            }
            
            if(already_gone.find(graph_dict[curr][i]) == already_gone.end())
            {
                float curr_val  = curr_g_cost[temp_check] + std::get<2>(graph_dict[curr][i]) + getEucledianDistance(std::get<0>(goal_tup), std::get<1>(goal_tup), std::get<0>(graph_dict[curr][i]), std::get<1>(graph_dict[curr][i]));
                if(curr_val < min_h)
                {
                    min_h = curr_val;
                    min_c = i; 
                }
                std::tuple<float, float> temp_w(std::get<0>(graph_dict[curr][i]), std::get<1>(graph_dict[curr][i]));
                curr_g_cost.insert({temp_w, (std::get<2>(graph_dict[curr][i]) + curr_g_cost[temp_check])});
                old_weight.insert({temp_w, std::get<2>(graph_dict[curr][i])});
                std::get<2>(graph_dict[curr][i]) = curr_val;
                visited_pq.push(graph_dict[curr][i]);
            }
            
        }
    }
	ROS_INFO_STREAM("Bye Bye " << curr_path.size());
}


void DeathStar::drawGraphonImage()
{
    if(graph.size() > 0)
    {
        cv::Mat img(605, 1006, CV_8UC3, cv::Scalar(0,0, 100));

        for(auto a: graph)
        {
            int x = (int)(a->getX()*10);
            int y = (int)(a->getY()*10);
            y = 605 - y;
            cv::Vec3b& color = img.at<cv::Vec3b>(y,x);
            color[0] = 255;
            /**
			for(auto N: a->neighbours)
            {
                int x1 = (int)(N->getX()*10);
                int y1 = (int)(N->getY()*10);
                y1 = 605 - y1;
                //cv::line(img, cv::Point(x,y), cv::Point(x1,y1), cv::Scalar(0,255, 0), 1);
            }
			**/
        }
        std::vector<std::tuple<float, float, float>> final = curr_path;
        int x = (int)(std::get<0>(final[0])*10);
        int y = (int)(std::get<1>(final[0])*10);
        y = 605 - y; 
        for(auto a: final)
        {
            ROS_INFO_STREAM(x << ", " << y);
            int x1 = (int)(std::get<0>(a)*10);
            int y1 = (int)(std::get<1>(a)*10);
            y1 = 605 - y1;
            cv::line(img, cv::Point(x,y), cv::Point(x1,y1), cv::Scalar(0,255, 255), 1);
            x = x1;
            y = y1;
        }
        cv::imshow("Graph", img);
        cv::waitKey(1000);
        cv::destroyAllWindows();
    }
    return;
}
