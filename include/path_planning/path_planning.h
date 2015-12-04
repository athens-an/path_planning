#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "path_planning/start.h"
#include "path_planning/goal.h"
#include "nav_msgs/GetMap.h"
#include <iostream>
#include <vector>

#include <nav_core/base_global_planner.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


struct cell {
	int currCellX;
	int currCellY;
	int fScore;
};

struct neighbour {
	int neighbourCellX;
	int neighbourCellY;
};


class Planner{
	
	ros::NodeHandle node;
	ros::Subscriber sub;
	ros::ServiceServer service1;
	ros::ServiceServer service2;
	ros::Timer timer;
	tf::TransformListener listener;
	
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
	bool goalCellValid(unsigned int goalCellX, unsigned int goalCellY);
	bool isFree(unsigned int ii, unsigned int jj);
	std::vector <int> findNeighbourValid (int currCellX, int currCellY);
	
	
	
	
	
	int getCellIndex(int ii, int jj);
	int calculateHScore(int currCell, int goalCell);
	bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
                             
    
	std::vector <int> path (int currCellX, int currCellY, int goalCellX, int goalCellY, float gScore[]);
	std::vector <cell> addToOpenList(int neighbourCellX, int neighbourCellY, float fScore);
	std::vector <int> initializePlanner (int currCellX, int currCellY, int goalCellX, int goalCellY);
	
	bool k;
	float goalCellX;
	float goalCellY;
	float currCellX;
	float currCellY;
	int height;
	int width;
	float resolution;
	int mapSize;
	int value;
			

};
	
