#include "path_planning/path_planning.h"


Planner::Planner()
{
	k = false;
	_sub = _node.subscribe("map", 1000, &Planner::readMap, this);
	_service2 = _node.advertiseService("start", &Planner::start, this);
	_service1 = _node.advertiseService("goal", &Planner::goal, this);
	//transform a point once every second
	_timer = _node.createTimer(ros::Duration(1.0), &Planner::currentPosition, this);

}

bool Planner::goal(path_planning::goalRequest &req, path_planning::goalResponse &res)
{
	ROS_INFO("goal");
	k = true;
	res.success = true;
	return true;
}

bool Planner::start(path_planning::startRequest &req, path_planning::startResponse &res)
{
	ROS_INFO("start");
	k = true;
	res.success = true;
	return true;
}

void Planner::calculatePath()
{
	while (k == false) {}
		
}

void Planner::currentPosition(const ros::TimerEvent& e)
{
//create a point in the robot0 frame to transform to the map frame
			ROS_INFO("CURRENT POSITION (pose_point)");

	try{
		
		tf::StampedTransform transform;
		_listener.lookupTransform("map", "robot0", ros::Time(0), transform);
	
		ROS_INFO("CURRENT POSITION (pose_point): (%.2f, %.2f, %.2f)",
		transform.getOrigin()[0], transform.getOrigin()[1], 0.0);
		
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
	}
}

void Planner::readMap(const nav_msgs::OccupancyGridConstPtr& msg)
{

	height = msg->info.height;
	width = msg->info.width;
	resolution = msg->info.resolution;
	map_size = height*width;
		
	//~ for (unsigned int ii = 0; ii <= map_size; ii++) {
		//~ ROS_INFO("oook %d", msg->data[ii]);
	//~ }	
		
	ROS_INFO("%d, %d, %f", height, width, resolution);
	
	
}

//elegxei an oi suntetagmenes tou keliou pou dothike anhkei ston xarth
bool Planner::isCellInMap(float x, float y)
{
	bool valid = true;
	
	if (x > (width * resolution) || y > (height * resolution)) {
		ROS_INFO("cell is not inside the map");
		valid = false;
	}

	return valid;
}

//metatrepei tis suntetagmenes x,y tou xarth se suntetagmenes keliou
void Planner::coordinateConvertToCell(float x, float y)
{
	float cell_x = x / resolution;
	float cell_y = y / resolution;

}

//metatrepei tis suntetagmenes x,y tou keliou se suntetagmenes tou xarth
void Planner::cellConvertToCoordinate(float x, float y)
{
	float m_x = x * resolution;
	float m_y = y * resolution;
	
}

//elegxei an oi suntetagmenes tou keliou einai empodio h oxi
bool Planner::goalCellValid(int goal_cell_x, int goal_cell_y) 
{
	bool valid = true;
	bool free_goal_cell = isFree(goal_cell_x, goal_cell_y);
	if (!free_goal_cell) {
		ROS_INFO("The goal cell is an obstacle");
		valid = false;
	}
	return valid;
}

//elegxei ean to keli einai eleuthero h oxi
bool Planner::isFree(int ii, int jj)
{
	bool valid = false;
	int cell_index = getCellIndex(ii, jj);
	if (cell_index == 0)
	{
		ROS_INFO("Cell is free");
		valid = true;
	}
	else 
	{
		ROS_INFO("Cell is an obstacle");
	}
	return valid;
}



//elegxei poia einai ta geitonika kelia kai krataei auta poy einai eleuthera
std::vector <cell> Planner::findNeighbourValid (int curr_cell_x, int curr_cell_y) 
{
	value = 0; //metraei tous geitones
	std::vector <cell> neighbour_cells_free; // disdiastatos ????
	cell N;
	
	for (unsigned int ii = -1; ii <= 1; ii++) {
		for (unsigned int jj = -1; jj <= 1; jj++) {
			if (!ii == 0 && !jj == 0) {
				if ((curr_cell_x + ii >= 0 && curr_cell_x + ii <= width) && (curr_cell_y + jj >= 0 && curr_cell_y + jj <= height)) {
					if (goalCellValid(curr_cell_x + ii, curr_cell_y + jj)) 
					{
						value ++;
						N.x = curr_cell_x + ii;
						N.y = curr_cell_y + jj;
						
						neighbour_cells_free.push_back(N);
					}
				}
			}
			else
			{
				ROS_INFO("NO NEIGHBOURS");
			}
		}
	}
	return neighbour_cells_free;
}


bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{

	if (isCellInMap(goal_cell_x, goal_cell_y) && goalCellValid(goal_cell_x, goal_cell_y)) 
	{
			findNeighbourValid(curr_cell_x, curr_cell_y);
	}
	else
	{
		ROS_INFO("THE GOAL CELL IS NOT INSIDE THE MAP OR IS AN OBSTACLE!");
	}
		
}

//lambanei to periexomeno tou keliou, 0 an einai eleuthero to keli, 100 an einai empodio kai -1 agnwsto
int Planner::getCellIndex(int ii, int jj) 
{
	int cell_index = 0;
	return cell_index;
}

//upologizetai to h(x) to opoio einai iso me thn apostash tou trexontos keliou apo ton teliko stoxo (gia oxi diagwnia)
int Planner::calculateHScore(int curr_cell_x, int curr_cell_y, int goal_cell_x, int goal_cell_y) 
{
	int h_score = abs(curr_cell_x - goal_cell_x) + abs(curr_cell_y - goal_cell_y);
	return h_score;
}

//arxikopoiw to gScore se enan pinaka
std::vector <int> Planner::initializePlanner (int curr_cell_x, int curr_cell_y, int goal_cell_x, int goal_cell_y)
{
	g_score = new int [map_size];
	for (unsigned int ii = 0; ii < map_size; ii ++) 
	{
		g_score[ii] = map_size + 1;
		
	}

}


std::vector <int> Planner::path (int curr_cell_x, int curr_cell_y, int goal_cell_x, int goal_cell_y, int g_score[])
{
	std::vector <cell> open_list;
	std::vector <cell> closed_list;
	std::vector <int> best_path;
	int possible_path;
	
	cell C;
	C.x = curr_cell_x;
	C.y = curr_cell_y;
	open_list.push_back(C);
	
	
	g_score[curr_cell_x][curr_cell_y] = 0; //gia to keli sto opoio vriskomaste
	h_score[curr_cell_x][curr_cell_y] = calculateHScore(curr_cell_x, curr_cell_y, goal_cell_x, goal_cell_y);
	f_score[curr_cell_x][curr_cell_y] = g_score[curr_cell_x][curr_cell_y] + h_score[curr_cell_x][curr_cell_y];
		
	while (!open_list.empty() && (g_score[goal_cell_x][goal_cell_y] == map_size + 1)) {
		
		std::vector <cell> neighbour_cell;
		neighbour_cell = findNeighbourValid(curr_cell_x, curr_cell_y);
		for (unsigned int ii = 0; ii < value; ii++)
		{
			
			if (g_score[neighbour_cell[ii].x][neighbour_cell[ii].y] == map_size + 1)
			{
				ROS_INFO("DEN TO EXEI EPISKEFTHEI AKOMA");
				g_score[neighbour_cell[ii].x][neighbour_cell[ii].y] = g_score[curr_cell_x][curr_cell_y] + 1;
				h_score[neighbour_cell[ii].x][neighbour_cell[ii].y] = calculateHScore(neighbour_cell[ii].x, neighbour_cell[ii].y, goal_cell_x, goal_cell_y);
				f_score[neighbour_cell[ii].x][neighbour_cell[ii].y] = g_score[neighbour_cell[ii].x][neighbour_cell[ii].y] + h_score[neighbour_cell[ii].x][neighbour_cell[ii].y];
				addToOpenList(neighbour_cell[ii].x, neighbour_cell[ii].y, f_score);
			}
			g_score[curr_cell_x][curr_cell_y] = g_score[neighbour_cell[ii].x][neighbour_cell[ii].y];
			
			
			
			//upologizei to kalutero monopati
			if (ii > 1) 
			{
				if (f_score[neighbour_cell[ii].x][neighbour_cell[ii].y] < f_score[neighbour_cell[ii - 1].x][neighbour_cell[ii - 1].y]) {
					possible_path = neighbour_cell[ii];
				}
				else {
					possible_path = neighbour_cell[ii-1];
				}
			}
			else {
				possible_path = neighbour_cell[ii];
			}	
			
		}
		best_path = path(possible_path, g_score);
		C.cell = possible_path;
		open_list.remove(C);
		closed_list.push_back(C);
		
	}
	
}

//prosthetw sth lista to geitoniko keli
std::vector <cell> Planner::addToOpenList(int neighbour_cell_x, int neighbour_cell_y, float f_score)
{
	std::vector <cell> open_list;
	cell C;
	C.x = neighbour_cell_x;
	C.y = neighbour_cell_y;
	C.f_score = f_score;
	open_list.push_back(C);
}	



