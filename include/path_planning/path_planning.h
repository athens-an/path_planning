#include <iostream>
#include <vector>
#include <limits>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "nav_msgs/GetMap.h"
#include "path_planning/start.h"
#include "path_planning/goal.h"



struct cell {
	int x;
	int y;
	int f_score;
};


class Planner{
	
	ros::NodeHandle _node;
	ros::Subscriber _sub;
	ros::ServiceServer _service1;
	ros::ServiceServer _service2;
	ros::Timer _timer;
	tf::TransformListener _listener;
	
	public:
	
	Planner();
	bool goal(path_planning::goalRequest &req, path_planning::goalResponse &res);
	bool start(path_planning::startRequest &req, path_planning::startResponse &res);
	void calculatePath();
	void currentPosition(const ros::TimerEvent& e);
	void readMap(const nav_msgs::OccupancyGridConstPtr& msg);
	
	bool isCellInMap(float x, float y);
	void coordinateConvertToCell(float x, float y);
	void cellConvertToCoordinate(float x, float y);
	bool goalCellValid(int goal_cell_x, int goal_cell_y);
	bool isFree(int ii, int jj);
	int getCellIndex(int ii, int jj);
	int calculateHScore(int curr_cell_x, int curr_cell_y, int goal_cell_x, int goal_cell_y);
	bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
 
	std::vector <cell> findNeighbourValid (int curr_cell_x, int curr_cell_y);
	std::vector <cell> path (int curr_cell_x, int curr_cell_y, int goal_cell_x, int goal_cell_y);
	
	bool k;
	float goal_cell_x;
	float goal_cell_y;
	float curr_cell_x;
	float curr_cell_y;
	int height;
	int width;
	float resolution;
	int map_size;
	int value;
	int h_score;
	
};
	
