#ifndef _PATH_PLANNING_H
#define _PATH_PLANNING_H

#include <iostream>
#include <vector>
#include <limits>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"

#include "nav_msgs/GetMap.h"
#include "nav_msgs/Path.h"
#include "path_planning/start.h"
#include "path_planning/goal.h"
#include <visualization_msgs/Marker.h>


struct cell {
	int x;
	int y;
	float f_score;
	int cf_x;
	int cf_y;
	int counter;
	
};


class Planner{
	
	ros::NodeHandle _node;
	ros::Subscriber _sub;
	ros::ServiceServer _service1;
	ros::ServiceServer _service2;
	ros::Timer _timer;
	ros::Publisher _pub;
	tf::TransformListener _listener;
	
	int * _index;
	float ** g_score;
	float ** f_score;
	
	int _goal_cell_x;
	int _goal_cell_y;
	int _goal_map_x;
	int _goal_map_y;
	
	int _curr_cell_x;
	int _curr_cell_y;
	int _new_curr_cell_x;
	int _new_curr_cell_y;
	int _height;
	int _width;
	float _resolution;
	int _map_size;
	
	std::vector <cell> _came_from;
	
	
	public:
	
	Planner();
	bool goal(path_planning::goalRequest &req, path_planning::goalResponse &res);
	bool start(path_planning::startRequest &req, path_planning::startResponse &res);
	void currentPosition(const ros::TimerEvent& e);
	void readMap(const nav_msgs::OccupancyGridConstPtr& msg);
	
	bool rightCell(int x, int y);
	int worldToMap(int w_coor);
	void mapToWorld(int m_x, int m_y);
	
	float calculateHScore(int curr_cell_x, int curr_cell_y, int _goal_cell_x, int _goal_cell_y);
	
	std::vector <cell> path (int _curr_cell_x, int _curr_cell_y, int _goal_cell_x, int _goal_cell_y);
	std::vector <cell> reconstructPath (const std::vector <cell>& _came_from, int _goal_cell_x, int _goal_cell_y);
	
	
};

#endif

