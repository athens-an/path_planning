#ifndef _NODE_H
#define _NODE_H


#include "path_planning/robot_perception.h"
#include "path_planning/graph.h"

#include <ros/ros.h>
#include <vector>
#include <limits>


struct node {
	int x;
	int y;
	int node_counter;
	int** connections;
	float** distance;
};


class Node {
	
	ros::NodeHandle _node;
	ros::Publisher _graph_pub;
	
	RobotPerception robot_perception;
	Graph graph_obj;
	
	int _step;
	
	std::vector <node> _neighbour_cell_test; //uniforms
	std::vector <node> _neighbour_cell; //uniforms

	int ** _create_graph;
	
	
	public:
	
	Node();
	void uniforms(int width, int height, float resolution, int map_size, 
					int curr_map_x, int curr_map_y, int goal_map_x, int goal_map_y);
	
	void createGraph(int map_size, float resolution);
	bool obstacleSearch(int x1, int y1, int x2, int y2);
	void visualGraph(int size, float resolution);

	
	int getNeighbourCellSize();
	int getNeighbourCellX(int ii);
	int getNeighbourCellY(int ii);
	int getNeighbourCellNodeCounter(int curr_map_x, int curr_map_y);
	int getStep();
	bool getObstacleSearch(int ii, int jj);
	float getMinDistance(int ii);
	bool getPixelGraph(int x, int y);
	
};

#endif