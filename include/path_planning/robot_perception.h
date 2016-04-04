#ifndef _ROBOT_PERCEPTION_H
#define _ROBOT_PERCEPTION_H


#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>

class RobotPerception {
	
	
	ros::NodeHandle _node;
	ros::Subscriber _map_sub;
	ros::Publisher _br_pub;
	
	ros::Timer _timer;
	
	tf::TransformListener _listener;
	
	int _width;
	int _height;
	int _map_size;
	float _resolution;
	
	int * _index;
	int * data;
	
	float ** _brushfire;
	
	float _curr_cell_x;
	float _curr_cell_y;
	float _curr_cell_z;
	
	double _yaw;
	
	float _goal_cell_x;
	float _goal_cell_y;
	
	
	
	public:
	
	RobotPerception();
	
	void readMap(const nav_msgs::OccupancyGridConstPtr& msg);
	int getMapWidth();
	int getMapHeight();
	int getMapSize();
	float getMapResolution();	
	
	void currentPosition(const ros::TimerEvent& e);
	float currentXPosition();
	float currentYPosition();
	float currentYaw();
	
	float goalXPosition();
	float goalYPosition();
	
	int worldToMap(int w_coor);
	void mapToWorld(int m_x, int m_y);
	
	bool rightCell(int x, int y);
	void brushfire();
	int getBrushfireCell(int x, int y);

};

#endif
