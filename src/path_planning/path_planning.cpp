#include "path_planning/path_planning.h"


Planner::Planner()
{	
	_service2 = _node.advertiseService("start", &Planner::start, this);
	_service1 = _node.advertiseService("goal", &Planner::goal, this);
	_sub = _node.subscribe("/map", 1000, &Planner::readMap, this);
	_pub = _node.advertise<nav_msgs::Path>("/move_base_simple/path", 10);
	//transform a point once every second
	_timer = _node.createTimer(ros::Duration(1.0), &Planner::currentPosition, this);
	
}


bool Planner::start(path_planning::startRequest &req, path_planning::startResponse &res)
{
	ROS_INFO_STREAM("start");
	res.success = true;
	return true;
}

void Planner::readMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
	_height = msg->info.height;
	_width = msg->info.width;
	_resolution = msg->info.resolution;
	_map_size = _height * _width;
	
	_index = new int[_map_size];
	
	for (unsigned int ii = 0; ii <= _map_size; ii++) 
	{
		_index[ii] = msg->data[ii];
	}
	
	
	
	ROS_INFO_STREAM("height, width, resolution: " << _height << " " << _width << " " << _resolution);
}

void Planner::currentPosition(const ros::TimerEvent& e)
{
	try
	{
		tf::StampedTransform transform;
		_listener.lookupTransform("/map", "robot0", ros::Time(0), transform);
		
		_curr_cell_x = transform.getOrigin()[0];
		_curr_cell_y = transform.getOrigin()[1];
		
		ROS_INFO_STREAM("CURRENT POSITION: " << _curr_cell_x << " " << _curr_cell_y << " " << 0.0);
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
	}
	
}



bool Planner::goal(path_planning::goalRequest &req, path_planning::goalResponse &res)
{
	ROS_INFO_STREAM("goal");
	//~ std::vector <cell> best_path;
	
	//~ _goal_cell_x = req.goal_cell_x; // world
	//~ _goal_cell_y = req.goal_cell_y;
	
	if (random())
	{
		res.success = true;
	}
	
	//~ _goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
	//~ _goal_map_y = worldToMap(_goal_cell_y);
	
	//~ ROS_INFO_STREAM("Pixel " << _goal_map_x << " " << _goal_map_y);
	//~ if (rightCell(_goal_map_x, _goal_map_y))
	//~ {
		//~ best_path = path(_curr_cell_x, _curr_cell_y, _goal_cell_x, _goal_cell_y);
	
		//~ ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y << "  and a goal: " << _goal_cell_x << " " << _goal_cell_y);
		
		//~ nav_msgs::Path plan;
		//~ geometry_msgs::PoseStamped pose;


		//~ plan.header.frame_id = "/map";
		
		
		//~ for (unsigned int ii = 0; ii < best_path.size(); ii ++)
		//~ {

			//~ // geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Point
			//~ pose.pose.position.x = best_path[ii].x;
			//~ pose.pose.position.y = best_path[ii].y;
			//~ pose.pose.position.z = 0.0;

			//~ // geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Quaternion
			//~ pose.pose.orientation.x = 0.0;
			//~ pose.pose.orientation.y = 0.0;
			//~ pose.pose.orientation.z = 0.0;
			//~ pose.pose.orientation.w = 1.0;

			//~ plan.poses.push_back(pose);
			
		//~ }
		//~ _pub.publish(plan);
		
		
		//~ res.success = true;
		
	//~ }
	else
	{
		ROS_INFO_STREAM("WRONG GOAL");
		res.success = false;
		return 1;
	}
	return true;
}


bool Planner::random()
{
	std::vector <cell> best_path;
	best_path.clear();
	
	int w = _width * _resolution;
	int h = _height * _resolution;
	_goal_cell_x = rand() % w;
	_goal_cell_y = rand() % h;
	
	ROS_INFO_STREAM("Goal cell " << _goal_cell_x << " " << _goal_cell_y);
	_goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
	_goal_map_y = worldToMap(_goal_cell_y);
	
	if (rightCell(_goal_map_x, _goal_map_y))
	{
		best_path = path(_curr_cell_x, _curr_cell_y, _goal_cell_x, _goal_cell_y);
		ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y << "  and a goal: " << _goal_cell_x << " " << _goal_cell_y);
		
		nav_msgs::Path plan;
		geometry_msgs::PoseStamped pose;


		plan.header.frame_id = "/map";
		
		
		for (unsigned int ii = 0; ii < best_path.size(); ii ++)
		{

			// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Point
			pose.pose.position.x = best_path[ii].x;
			pose.pose.position.y = best_path[ii].y;
			pose.pose.position.z = 0.0;

			// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Quaternion
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;

			plan.poses.push_back(pose);
			
		}
		_pub.publish(plan);
		
		return true;
	}
	else
	{
		return false;
	}
}




//elegxei an oi suntetagmenes tou keliou pou dothike anhkei ston xarth
bool Planner::rightCell(int x, int y)
{
	bool valid = true;
	
	if (x >= _width || y >= _height || x < 0 || y < 0) 
	{
		valid = false;
	}
	else
	{
		int b = y * _width + x;
		//~ (2,2) = y * _width + 2
		//~ (1,4) = y * _width + 1
		//~ (4,1) = y * _width + 4
		//~ ROS_INFO_STREAM("index " << _index[b]);
		if (_index[b] == 0)
		{
			//~ ROS_INFO_STREAM(" FREE ");			
		}
		else if (_index[b] == 100)
		{
			//~ ROS_INFO_STREAM(" OBSTACLE ");
			valid = false;
		}
		else
		{
			//~ ROS_INFO_STREAM(" UNKNOWN ");
			valid = false;
		}
	}
	return valid;
}

//metatrepei tis suntetagmenes x,y tou xarth se suntetagmenes keliou
int Planner::worldToMap(int w_coor)
{
	int m_coor = w_coor / _resolution;
	return m_coor;
}

//metatrepei tis suntetagmenes x,y tou keliou se suntetagmenes tou xarth
void Planner::mapToWorld(int m_x, int m_y)
{
	int w_x = m_x * _resolution;
	int w_y = m_y * _resolution;
}

//upologizetai to h(x) to opoio einai iso me thn apostash tou trexontos keliou apo ton teliko stoxo (gia oxi diagwnia)
float Planner::calculateHScore (int curr_cell_x, int curr_cell_y, int _goal_cell_x, int _goal_cell_y) 
{
	float h_score = abs(curr_cell_x - _goal_cell_x) + abs(curr_cell_y - _goal_cell_y);
	//~ ROS_INFO_STREAM("Hscore " << h_score);
	return h_score;
}

std::vector <cell> Planner::path (int _curr_cell_x, int _curr_cell_y, int _goal_cell_x, int _goal_cell_y)
{
	
	std::vector <cell> open_list;
	std::vector <cell> closed_list;
	std::vector <cell> best_path;

	open_list.clear();
	closed_list.clear();
	best_path.clear();
	_came_from.clear();
	
	cell C;
	cell N_C;
	cell C_F;
	
	g_score = new float *[_width];
	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		g_score[ii] = new float [_height];
		for (unsigned int jj = 0; jj < _height; jj ++) 
		{
			g_score[ii][jj] = _map_size + 1;
		}
	}
	
	f_score = new float *[_width];
	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		f_score[ii] = new float [_height];
		for (unsigned int jj = 0; jj < _height; jj ++) 
		{
			f_score[ii][jj] = std::numeric_limits<float>::infinity();
		}
	}
	
	//prosthetw thn trexousa thesh (keli) sthn open list
	C.x = _curr_cell_x;
	C.y = _curr_cell_y;
	open_list.push_back(C);
	
	int c_f_counter = 0; // gia na kserei apo poio keli egine h eksaplwsh
	int g_counter = 0; // counter gia ta g_score
	int counter = 0; // gia na kserei poio keli tha diagrapsei apo thn open list
	float infinity = std::numeric_limits<float>::infinity();
	
	
	g_score[_curr_cell_x][_curr_cell_y] = 0; //gia to keli sto opoio vriskomaste
	float h_score = calculateHScore(_curr_cell_y, _curr_cell_y, _goal_cell_x, _goal_cell_y);
	f_score[_curr_cell_x][_curr_cell_y] = g_score[_curr_cell_x][_curr_cell_y] + h_score;
	
	
	while (!open_list.empty() && !(_curr_cell_x == _goal_cell_x && _curr_cell_y == _goal_cell_y))
	{		

		std::vector <cell> neighbour_cell;
		neighbour_cell.clear();
		
		open_list.erase(open_list.begin() + counter);
		closed_list.push_back(C);
		
		
		//~ for (unsigned int ii = 0; ii < closed_list.size(); ii ++)
		//~ {
			//~ ROS_INFO_STREAM(" closed list " << closed_list[ii].x << " " << closed_list[ii].y);
		//~ }
		//~ 
		//~ for (unsigned int ii = 0; ii < _came_from.size(); ii ++)
		//~ {
			//~ ROS_INFO_STREAM(" came from list " << _came_from[ii].x << " " << _came_from[ii].y);
		//~ }
		
		float min = infinity;
		
		for (unsigned int ii = 1; ii <= 3; ii ++) 
		{
			for (unsigned int jj = 1; jj <= 3; jj ++) 
			{
				//metatrepei tis suntetagmenes
				int curr_map_x = worldToMap(_curr_cell_x + ii - 2);
				int curr_map_y = worldToMap(_curr_cell_y + jj - 2);
				
				//~ ROS_INFO_STREAM("neighbour " << _curr_cell_x + ii - 2 << " " << _curr_cell_y + jj - 2);
				
				//dexetai mono ta panw-katw deksia-aristera kelia
				if ((_curr_cell_x + ii - 2 - _curr_cell_x) == -1 && (_curr_cell_y + jj - 2 - _curr_cell_y) == -1
					|| (_curr_cell_x + ii - 2 - _curr_cell_x) == -1 && (_curr_cell_y + jj - 2 - _curr_cell_y) == 1
					|| (_curr_cell_x + ii - 2 - _curr_cell_x) == 1 && (_curr_cell_y + jj - 2 - _curr_cell_y) == -1
					|| (_curr_cell_x + ii - 2 - _curr_cell_x) == 1 && (_curr_cell_y + jj - 2 - _curr_cell_y) == 1
					|| (ii -2) == 0 && (jj - 2) == 0)
				{
					//~ ROS_INFO_STREAM("No Neighbours");
				}
				else
				{
					//elegxw an einai mesa ston xarti kai an einai eleuthero h oxi
					if (rightCell(curr_map_x, curr_map_y)) 
					{
						//elegxw an to exei ksanaepiskeuthei
						if (g_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2] == _map_size + 1) //den exei epektathei akoma
						{
							//~ ROS_INFO_STREAM("Not visited yet");
							g_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2] = g_score[_curr_cell_x][_curr_cell_y] + 1;
							h_score = calculateHScore(_curr_cell_x + ii - 2, _curr_cell_y + jj - 2, _goal_cell_x, _goal_cell_y);
							f_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2] = g_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2] + h_score;
							//~ ROS_INFO_STREAM("Gscore new " << g_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2]);
							//pernaei sthn open list ta geitonika diathesima kelia
							N_C.x = _curr_cell_x + ii - 2;
							N_C.y = _curr_cell_y + jj - 2;
							N_C.f_score = f_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2];
							N_C.cf_x = _curr_cell_x; //apo auto egine h epektash
							N_C.cf_y = _curr_cell_y;
							N_C.counter = c_f_counter;
							open_list.push_back(N_C);
					
							//~ ROS_INFO_STREAM("Fscore " << f_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2]);
							
			
						}
						else //to exei ksanaepiskeuthei
						{
							for (unsigned int zz = 0; zz < open_list.size(); zz ++)
							{
								if (_curr_cell_x + ii - 2 == open_list[zz].x && _curr_cell_y + jj - 2 == open_list[zz].y)
								{
									if (g_score[_curr_cell_x][_curr_cell_y] + 1 < g_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2])
									{
										g_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2] = g_score[_curr_cell_x][_curr_cell_y] + 1;
										f_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2] = g_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2] + 
												calculateHScore(_curr_cell_x + ii - 2, _curr_cell_y + jj - 2, _goal_cell_x, _goal_cell_y);
										C.f_score = f_score[_curr_cell_x + ii - 2][_curr_cell_y + jj - 2];
										C.cf_x = _curr_cell_x;
										C.cf_y = _curr_cell_y;
										C.counter = c_f_counter;
									}
								}
							}
							ROS_INFO_STREAM("Has visited");
						}
					}
				}
			}
		}
				
		//upologizei to mikrotero f_score apo ta kelia pou exei episkeuthei
		for (unsigned int zz = 0; zz < open_list.size(); zz ++)
		{
			if (min > open_list[zz].f_score)
			{
				min = open_list[zz].f_score;
				_curr_cell_x = open_list[zz].x;
				_curr_cell_y = open_list[zz].y;
				counter = zz;
			}
		}
		
		//~ ROS_INFO_STREAM("MIN " << min);
		
		
		ROS_INFO_STREAM("Next cell " << _curr_cell_x << " " << _curr_cell_y);
		C.x = _curr_cell_x;
		C.y = _curr_cell_y;

		C_F.x = open_list[counter].cf_x;
		C_F.y = open_list[counter].cf_y;
		C_F.counter = open_list[counter].counter;
		c_f_counter ++ ;
		_came_from.push_back(C_F);
	
	}
	
	C.x = _goal_cell_x;
	C.y = _goal_cell_y;
	C.counter = _came_from.size();
	closed_list.push_back(C);
	_came_from.push_back(C);
	
	best_path = reconstructPath(_came_from, _goal_cell_x, _goal_cell_y);
	
	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		delete [] g_score[ii];
		delete [] f_score[ii];
	}
	delete [] g_score;
	delete [] f_score;
	//~ delete [] _index;
	
	return best_path;
}

std::vector <cell> Planner::reconstructPath (const std::vector <cell>& _came_from, int _goal_cell_x, int _goal_cell_y)
{
	std::vector <cell> best_path;
	
	int count = _came_from.size();
	cell B;

	ROS_INFO_STREAM("size of vector " << count);
	int current_x = _goal_cell_x;
	int current_y = _goal_cell_y;
	ROS_INFO_STREAM("current " << _curr_cell_x << " " << _curr_cell_y);
	
	
	while (!(count == 0))
	{
		current_x = _came_from[count - 1].x;
		current_y = _came_from[count - 1].y;
		count = _came_from[count - 1].counter;
		
		ROS_INFO_STREAM("reConstructorPath " << current_x << " " << current_y);
		B.x = current_x;
		B.y = current_y;
		best_path.push_back(B);
		ROS_INFO_STREAM("count " << count);
		
	}

	return best_path;
}









