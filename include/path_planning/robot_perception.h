#ifndef _ROBOT_PERCEPTION_H
#define _ROBOT_PERCEPTION_H


#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <stdr_msgs/RfidSensorMeasurementMsg.h>
#include <stdr_msgs/RfidTagVector.h>
#include <stdr_msgs/RfidTag.h>
#include <fstream>
#include <sstream>
#include <iterator>



class RobotPerception {
	
	ros::NodeHandle _node;
	ros::Subscriber _map_sub;
	ros::Publisher _br_pub;
	
	int _width;
	int _height;
	int _map_size;
	float _resolution;
	
	int * _index;
	int * data;
	float ** _brushfire;
	
	
	float _goal_cell_x;
	float _goal_cell_y;
	
	
	public:
	
	RobotPerception();
	
	void readMap(const nav_msgs::OccupancyGridConstPtr& msg);
	int getMapWidth();
	int getMapHeight();
	int getMapSize();
	float getMapResolution();
	int getMapCell(int ii, int jj); 
	int* getMapIndex();
	
	//~ void currentPosition(const ros::TimerEvent& e);
	//~ float getCurrentXPosition();
	//~ float getCurrentYPosition();
	//~ float getCurrentYaw();
	
	float getGoalXPosition();
	float getGoalYPosition();
	
	int worldToMap(int w_coor);
	void mapToWorld(int m_x, int m_y);
	
	bool rightCell(int x, int y);
	void brushfire();
	int getBrushfireCell(int x, int y);
	
	
};

#endif
