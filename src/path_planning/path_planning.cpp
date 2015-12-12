#include "path_planning/path_planning.h"


Planner::Planner()
{	
	_service2 = _node.advertiseService("start", &Planner::start, this);
	_service1 = _node.advertiseService("goal", &Planner::goal, this);
	_sub = _node.subscribe("map", 1000, &Planner::readMap, this);
	
	//transform a point once every second
	_timer = _node.createTimer(ros::Duration(1.0), &Planner::currentPosition, this);

}

bool Planner::goal(path_planning::goalRequest &req, path_planning::goalResponse &res)
{
	ROS_INFO("goal");
	std::vector <cell> best_path;
	
	int _goal_cell_x = req.goal_cell_x;
	int _goal_cell_y = req.goal_cell_y;
		
	if (rightCell(_goal_cell_x, _goal_cell_y))
	{
		best_path = path(_curr_cell_x, _curr_cell_y, _goal_cell_x, _goal_cell_y);
		res.success = true;
	}
	else
	{
		ROS_INFO("WRONG GOAL");
		res.success = false;
		return 1;
	}
	return true;
}

bool Planner::start(path_planning::startRequest &req, path_planning::startResponse &res)
{
	ROS_INFO("start");
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
		
	ROS_INFO("%d, %d, %f", _height, _width, _resolution);

}

void Planner::currentPosition(const ros::TimerEvent& e)
{
	try
	{
		
		tf::StampedTransform transform;
		_listener.lookupTransform("map", "robot0", ros::Time(0), transform);
		
		_curr_cell_x = transform.getOrigin()[0];
		_curr_cell_y = transform.getOrigin()[1];
		
		ROS_INFO("CURRENT POSITION: (%.2f, %.2f, %.2f)", _curr_cell_x, _curr_cell_y, 0.0);
		
		
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
	}
}

//elegxei an oi suntetagmenes tou keliou pou dothike anhkei ston xarth
bool Planner::rightCell(float x, float y)
{
	bool valid = true;
	
	if (x > (_width * _resolution) || y > (_height * _resolution)) 
	{
		valid = false;
	}
	else
	{
		int b = x * _width + y;
	
		//~ (2,2) = y * _width + 2
		//~ (1,4) = y * _width + 4
		//~ (4,1) = y * _width + 1
	
		if (_index[b] == 0)
		{
			ROS_INFO("FREE");
		}
		else if (_index[b] == 100)
		{
			ROS_INFO("OBSTACLE");
			valid = false;
		}
		else
		{
			ROS_INFO("UNKNOWN");
			valid = false;
		}
	}

	return valid;
}

//metatrepei tis suntetagmenes x,y tou xarth se suntetagmenes keliou
void Planner::coordinateConvertToCell(float x, float y)
{
	float cell_x = x / _resolution;
	float cell_y = y / _resolution;

}

//metatrepei tis suntetagmenes x,y tou keliou se suntetagmenes tou xarth
void Planner::cellConvertToCoordinate(float x, float y)
{
	float m_x = x * _resolution;
	float m_y = y * _resolution;
	
}



std::vector <cell> Planner::path (int _curr_cell_x, int _curr_cell_y, int _goal_cell_x, int _goal_cell_y)
{
	std::vector <cell> open_list;
	std::vector <cell> closed_list;
	std::vector <cell> neighbour_cell;
	
	cell C;
	cell N_C;
	
	//prosthetw thn trexousa thesh sthn open list
	C.x = _curr_cell_x;
	C.y = _curr_cell_y;
	open_list.push_back(C);
	
	int counter = 1;
	
	double infinity = std::numeric_limits<double>::infinity();
	
	int ** g_score;
	g_score = new int *[_width];

	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		g_score[ii] = new int [_height];
		for (unsigned int jj = 0; jj < _height; jj ++) 
		{
			g_score[ii][jj] = _map_size * _map_size;
		}
	}
	
	
	int ** f_score ;
	f_score = new int *[_width];
	
	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		f_score[ii] = new int [_height];
		for (unsigned int jj = 0; jj < _height; jj ++) 
		{
			f_score[ii][jj] = infinity;
		}
	}
	
	g_score[_curr_cell_x][_curr_cell_y] = 0; //gia to keli sto opoio vriskomaste
	int h_score = calculateHScore(_curr_cell_x, _curr_cell_y, _goal_cell_x, _goal_cell_y);
	f_score[_curr_cell_x][_curr_cell_y] = g_score[_curr_cell_x][_curr_cell_y] + h_score;
	
		
	while (!open_list.empty() && (g_score[_goal_cell_x][_goal_cell_y] == _map_size * _map_size)) 
	{
		//krataei to twrino keli sto C gia na prostethei meta sthn closed list
		C.x = _curr_cell_x;
		C.y = _curr_cell_y;
		
		for (unsigned int ii = 1; ii <= 3; ii ++) 
		{
			for (unsigned int jj = 1; jj <= 3; jj ++) 
			{
				if (!(ii - 2) == 0 && !(jj - 2) == 0) 
				{
					if ((_curr_cell_x + ii - 2 >= 0 && _curr_cell_x + ii - 2 <= _width) && (_curr_cell_y + jj - 2 >= 0 && _curr_cell_y + jj - 2 <= _height)) 
					{
						if (rightCell(_curr_cell_x + ii - 2, _curr_cell_y + jj - 2)) 
						{
							N_C.x = _curr_cell_x + ii - 2;
							N_C.y = _curr_cell_y + jj - 2;
							neighbour_cell.push_back(N_C); //krataei se pinaka ta diathesima kelia
						}
					}
				}
				else
				{
					ROS_INFO("NO NEIGHBOURS");
				}
			}
		}
		
		
		for (unsigned int ii = 1; ii <= neighbour_cell.size(); ii ++)
		{
			if (g_score[neighbour_cell[ii].x][neighbour_cell[ii].y] == _map_size * _map_size) //dhladh den exei episkeuthei akoma
			{
				ROS_INFO("DEN TO EXEI EPISKEFTHEI AKOMA");
				g_score[neighbour_cell[ii].x][neighbour_cell[ii].y] = g_score[_curr_cell_x][_curr_cell_y] + 1;
				h_score = calculateHScore(neighbour_cell[ii].x, neighbour_cell[ii].y, _goal_cell_x, _goal_cell_y);
				f_score[neighbour_cell[ii].x][neighbour_cell[ii].y] = g_score[neighbour_cell[ii].x][neighbour_cell[ii].y] + h_score;
				
				//pernaei sthn open list ta geitonika diathesima kelia
				N_C.x = neighbour_cell[ii].x;
				N_C.y = neighbour_cell[ii].y;
				N_C.f_score = f_score[neighbour_cell[ii].x][neighbour_cell[ii].y];
				open_list.push_back(N_C);
				
			}
			else
			{
				ROS_INFO("TO EXEI EPISKEFTHEI");
			}
			
		}
		
		int min = infinity;
		
		for (unsigned int ii = 1; ii <= neighbour_cell.size(); ii ++)
		{
			//upologizei to kalutero monopati

			if (min > f_score[neighbour_cell[ii].x][neighbour_cell[ii].y])
			{
				min = f_score[neighbour_cell[ii].x][neighbour_cell[ii].y];
				N_C.x = neighbour_cell[ii].x;
				N_C.y = neighbour_cell[ii].y;
				counter += ii;
			}
		}
		
		_curr_cell_x = N_C.x;
		_curr_cell_y = N_C.y;
		ROS_INFO("trexon %d %d", _curr_cell_x, _curr_cell_y);
			
		open_list.erase(open_list.begin() + counter);
		closed_list.push_back(C);
		

		counter += neighbour_cell.size();
		
	}
	
	for (unsigned int ii = 0; ii < _width; ii ++) 
	{
		delete [] g_score[ii];
		delete [] f_score[ii];
	}
	delete [] g_score;
	delete [] f_score;
	//~ delete [] _index;
	
}

//upologizetai to h(x) to opoio einai iso me thn apostash tou trexontos keliou apo ton teliko stoxo (gia oxi diagwnia)
int Planner::calculateHScore (int _curr_cell_x, int _curr_cell_y, int _goal_cell_x, int _goal_cell_y) 
{
	int h_score = abs(_curr_cell_x - _goal_cell_x) + abs(_curr_cell_y - _goal_cell_y);
	ROS_INFO("H_SCORE %d", h_score);
	return h_score;
}


