#include "path_planning/path_planning.h"


Planner::Planner()
{	
	_service2 = _node.advertiseService("start", &Planner::start, this);
	//~ _service1 = _node.advertiseService("goal", &Planner::goal, this);
	_sub = _node.subscribe("map", 1000, &Planner::readMap, this);
	
	//transform a point once every second
	//~ _timer = _node.createTimer(ros::Duration(1.0), &Planner::currentPosition, this);
}

bool Planner::start(path_planning::startRequest &req, path_planning::startResponse &res)
{
	ROS_INFO_STREAM("start");
	res.success = true;
	return true;
}

void Planner::readMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
	_height = 10;
	_width = 10;
	_resolution = 0.3;
	_map_size = _height * _width;
	
	//~ _index = new int[_map_size];
	//~ for (unsigned int ii = 0; ii <= _map_size; ii++) 
	//~ {
		//~ _index[ii] = msg->data[ii];
	//~ }
	
	std::vector <cell> best_path;
	_test = new float *[10];
	for (unsigned int ii = 0; ii < 10; ii ++)
	{
		_test[ii] = new float [10];
		for (unsigned int jj = 0; jj < 10; jj ++)
		{
			if ((ii == 1 && jj == 0) || (ii == 1 && jj == 3) || (ii == 3 && jj == 1) || (ii == 3 && jj == 2) || (ii == 3 && jj == 4))
			{
				_test[ii][jj] = 100;
			}
			else if ((ii == 0 && jj == 2) || (ii == 2 && jj == 4) || (ii == 4 && jj == 2))
			{
				_test[ii][jj] = -1;
			}
			else
			{
				_test[ii][jj] = 0;
			}
		}
	}

	_curr_cell_x = 0;
	_curr_cell_y = 0;
	_goal_cell_x = 2;
	_goal_cell_y = 2;
	
	_goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
	_goal_map_y = worldToMap(_goal_cell_y);
	
	if (rightCell(_goal_map_x, _goal_map_y))
	{
		best_path = path(_curr_cell_x, _curr_cell_y, _goal_cell_x, _goal_cell_y);
	}
	
	ROS_INFO_STREAM("height, width, resolution: " << _height << " " << _width << " " << _resolution);
}

//~ void Planner::currentPosition(const ros::TimerEvent& e)
//~ {
	//~ try
	//~ {
		//~ tf::StampedTransform transform;
		//~ _listener.lookupTransform("map", "robot0", ros::Time(0), transform);
		//~ 
		//~ _curr_cell_x = transform.getOrigin()[0];
		//~ _curr_cell_y = transform.getOrigin()[1];
		//~ 
		//~ ROS_INFO_STREAM("CURRENT POSITION: " << _curr_cell_x << " " << _curr_cell_y << " " << 0.0);
	//~ }
	//~ catch(tf::TransformException& ex){
		//~ ROS_ERROR("%s",ex.what());
        //~ ros::Duration(1.0).sleep();
	//~ }
//~ }

//~ bool Planner::goal(path_planning::goalRequest &req, path_planning::goalResponse &res)
//~ {
	//~ ROS_INFO_STREAM("goal");
	//~ std::vector <cell> best_path;
	//~ 
	//~ _goal_cell_x = req.goal_cell_x; // world
	//~ _goal_cell_y = req.goal_cell_y;
	//~ 
	//~ _goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
	//~ _goal_map_y = worldToMap(_goal_cell_y);
	//~ 
	//~ ROS_INFO_STREAM("Pixel " << _goal_map_x << " " << _goal_map_y);
	//~ 
	//~ if (rightCell(_goal_map_x, _goal_map_y))
	//~ {
		//~ best_path = path(_curr_cell_x, _curr_cell_y, _goal_cell_x, _goal_cell_y);
		//~ res.success = true;
	//~ }
	//~ else
	//~ {
		//~ ROS_INFO_STREAM("WRONG GOAL");
		//~ res.success = false;
		//~ return 1;
	//~ }
	//~ return true;
//~ }

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
		ROS_INFO_STREAM("index " << _test[x][y]);
		if (_test[x][y] == 0)
		{
			ROS_INFO_STREAM(" FREE ");			
		}
		else if (_test[x][y] == 100)
		{
			ROS_INFO_STREAM(" OBSTACLE ");
			valid = false;
		}
		else
		{
			ROS_INFO_STREAM(" UNKNOWN ");
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
	ROS_INFO_STREAM("Hscore " << h_score);
	return h_score;
}

std::vector <cell> Planner::path (int _curr_cell_x, int _curr_cell_y, int _goal_cell_x, int _goal_cell_y)
{
	std::vector <cell> open_list;
	std::vector <cell> closed_list;
	
	cell C;
	cell N_C;
	
	//prosthetw thn trexousa thesh (keli) sthn open list
	C.x = _curr_cell_x;
	C.y = _curr_cell_y;
	open_list.push_back(C);
	
	int g_counter = 0;
	int counter = 0;
	float infinity = std::numeric_limits<float>::infinity();
	
	float ** g_score;
	g_score = new float *[_width];

	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		g_score[ii] = new float [_height];
		for (unsigned int jj = 0; jj < _height; jj ++) 
		{
			//~ g_score[ii][jj] = _map_size * _map_size;
			g_score[ii][jj] = _map_size + 1;
		}
	}
	
	g_score[_curr_cell_x][_curr_cell_y] = 0; //gia to keli sto opoio vriskomaste
	float h_score = calculateHScore(_curr_cell_y, _curr_cell_y, _goal_cell_x, _goal_cell_y);
	float f_score = g_score[_curr_cell_x][_curr_cell_y] + h_score;
	
		
	while (!open_list.empty() && (g_score[_goal_cell_x][_goal_cell_y] == _map_size + 1)) 
	{
		std::vector <cell> neighbour_cell;

		//krataei to twrino keli sto C gia na prostethei meta sthn closed list
		if (counter != 0)
		{
			C.x = _curr_cell_x;
			C.y = _curr_cell_y;
			ROS_INFO_STREAM("NAI");
		}
		
		for (unsigned int ii = 1; ii <= 3; ii ++) 
		{
			for (unsigned int jj = 1; jj <= 3; jj ++) 
			{
				//metatrepei tis suntetagmenes
				int curr_map_x = worldToMap(_curr_cell_x + ii - 2);
				int curr_map_y = worldToMap(_curr_cell_y + jj - 2);
				
				ROS_INFO_STREAM("neighbour (pixel) " << curr_map_x << " " << curr_map_y);
				if ((_curr_cell_x + ii - 2 - _curr_cell_x) == -1 && (_curr_cell_y + jj - 2 - _curr_cell_y) == -1
					|| (_curr_cell_x + ii - 2 - _curr_cell_x) == -1 && (_curr_cell_y + jj - 2 - _curr_cell_y) == 1
					|| (_curr_cell_x + ii - 2 - _curr_cell_x) == 1 && (_curr_cell_y + jj - 2 - _curr_cell_y) == -1
					|| (_curr_cell_x + ii - 2 - _curr_cell_x) == 1 && (_curr_cell_y + jj - 2 - _curr_cell_y) == 1
					|| (ii -2) == 0 && (jj - 2) == 0)
				{
					ROS_INFO_STREAM("No Neighbours");
				}
				else
				{
					if ((curr_map_x >= 0 && curr_map_x < _width) && (curr_map_y >= 0 && curr_map_y < _height)) 
					{
						if (rightCell(curr_map_x, curr_map_y)) 
						{
							N_C.x = _curr_cell_x + ii - 2;
							N_C.y = _curr_cell_y + jj - 2;
							ROS_INFO_STREAM("Free neighbour " << N_C.x << " " << N_C.y);
							neighbour_cell.push_back(N_C); //krataei se pinaka tous diathesimous geitones
						}
					}
			
				}
			}
		}
		
		ROS_INFO_STREAM("size " << neighbour_cell.size());
		
		float min = infinity;
		int list_counter = counter;
		
		for (unsigned int ii = 1; ii <= neighbour_cell.size(); ii ++)
		{
			ROS_INFO_STREAM("cell " << neighbour_cell[ii - 1].x << " " << neighbour_cell[ii - 1].y);
			
			if (g_score[neighbour_cell[ii - 1].x][neighbour_cell[ii - 1].y] == _map_size + 1) //dhladh den exei episkeuthei akoma
			{
				ROS_INFO_STREAM("Not visited yet");
				g_score[neighbour_cell[ii - 1].x][neighbour_cell[ii - 1].y] = g_counter + 1;
				h_score = calculateHScore(neighbour_cell[ii- 1].x, neighbour_cell[ii - 1].y, _goal_cell_x, _goal_cell_y);
				f_score = g_score[neighbour_cell[ii - 1].x][neighbour_cell[ii - 1].y] + h_score;
				ROS_INFO_STREAM("Gscore new " << g_score[neighbour_cell[ii - 1].x][neighbour_cell[ii - 1].y]);

				//pernaei sthn open list ta geitonika diathesima kelia se pixel
				N_C.f_score = f_score;
				open_list.push_back(N_C);
				
				g_counter ++;
				
				ROS_INFO_STREAM("Fscore " << f_score);
				if (min > f_score)
				{
					min = f_score;
					N_C.x = neighbour_cell[ii - 1].x;
					N_C.y = neighbour_cell[ii - 1].y;
					list_counter += ii;
					ROS_INFO_STREAM("counter1 " << list_counter);
				}
				
			}
			else
			{
				ROS_INFO_STREAM("Has visited");
			}
		}
		

		ROS_INFO_STREAM("MIN " << min);
		
		_curr_cell_x = N_C.x;
		_curr_cell_y = N_C.y;
		ROS_INFO_STREAM("Current cell " << _curr_cell_x << " " << _curr_cell_y);
		counter = counter + neighbour_cell.size();
		ROS_INFO_STREAM("counter " << counter);
			
		open_list.erase(open_list.begin() + list_counter);
		closed_list.push_back(C);
		
	}

	
	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		delete [] g_score[ii];
		delete [] _test[ii];
	}
	delete [] g_score;
	delete [] _test;
	
}



