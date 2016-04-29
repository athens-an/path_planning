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
	
	
	
	
	ros::Subscriber _laser_sub;
	ros::Subscriber _rfid_reader_sub;
	ros::Subscriber _rfid_tags_sub;
	std::string _map_topic_param;
	std::string _laser_topic_param;
	std::string _rfid_reader_topic_param;
	std::string _rfid_tags_topic_param;
	std::vector<float> _laser_ranges;
	std::vector<std::string> _rfid_ids;
	std::vector<std::string> _rfid_msgs;
	std::vector<std::vector<float> > _rfid_pose;
	std::vector<stdr_msgs::RfidTag> _rfid_tags;
	std::vector<std::string> _rfid_tags_id;
	std::vector<float> _rfid_tags_x;
	std::vector<float> _rfid_tags_y;
	float _max_range;
	float _increment;
	float _angle_min;
	
	
	
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
	
	
	void laserRangesCallback(sensor_msgs::LaserScan laser_scan_msg);
	void RfidReaderCallback (stdr_msgs::RfidSensorMeasurementMsg rfid_reader_msg);
	void RfidTagsCallback (stdr_msgs::RfidTagVector rfid_tag_msg);
	void RfidPose();
	std::vector<float> getLaserRanges();
	float getRangeMax();
	float getAngleIncrement();
	float getAngleMin();
	std::vector<std::string> getRfidIds();
	std::vector<std::string> getRfidMsgs();
	std::vector<std::vector<float> > getRfidPose();
	int getMapCell(int ii, int jj);
	int* getMapIndex();
	
	
	

};

#endif
