#ifndef _NODE_H
#define _NODE_H


#include "path_planning/robot_perception.h"

#include <ros/ros.h>
#include <vector>



struct node {
	int x;
	int y;
	int connection;
};



class Node {
	
	ros::NodeHandle _node;
	
	RobotPerception robot_perception;
	
	int _step;
	
	public:
	
	Node();
	void uniforms();
};

#endif
