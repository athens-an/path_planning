#include "path_planning/path_planning.h"
	
Planner::Planner()
{
	k = false;
	sub = node.subscribe("map", 1000, &Planner::readMap, this);
	service2 = node.advertiseService("start", &Planner::start, this);
	service1 = node.advertiseService("goal", &Planner::goal, this);
	//transform a point once every second
	timer = node.createTimer(ros::Duration(1.0), &Planner::currentPosition, this);

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
		listener.lookupTransform("map", "robot0", ros::Time(0), transform);
	
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
	mapSize = height*width;
		
	//~ for (unsigned int ii=0; ii<=width*height; ii++) {
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
	float cellX = x / resolution;
	float cellY = y / resolution;

}

//metatrepei tis suntetagmenes x,y tou keliou se suntetagmenes tou xarth
void Planner::cellConvertToCoordinate(float x, float y)
{
	float mx = x * resolution;
	float my = y * resolution;
	
}

//elegxei an oi suntetagmenes tou keliou einai empodio h oxi
bool Planner::goalCellValid(unsigned int goalCellX, unsigned int goalCellY) 
{
	bool valid = true;
	bool freeGoalCell = isFree(goalCellX, goalCellY);
	if (!freeGoalCell) {
		ROS_INFO("The goal cell is an obstacle");
		valid = false;
	}
	return valid;
}

//elegxei ean to keli einai eleuthero h oxi
bool Planner::isFree(unsigned int ii, unsigned int jj)
{
	bool valid = false;
	int cellIndex = getCellIndex(ii, jj);
	if (cellIndex == 0)
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
std::vector <int> Planner::findNeighbourValid (int currCellX, int currCellY) 
{
	value = 0; //metraei tous geitones
	std::vector <int> neighbourCellsFree; // dusdiastatos ????
	
	for (unsigned int ii = -1; ii <= 1; ii++) {
		for (unsigned int jj = -1; jj <= 1; jj++) {
			if (!ii == 0 && !jj == 0) {
				if ((currCellX + ii >= 0 && currCellX + ii <= width) && (currCellY + jj >= 0 && currCellY + jj <= height)) {
					if (goalCellValid(currCellX + ii, currCellY + jj)) 
					{
						value ++;
						neighbourCellsFree.push_back(currCellX + ii, currCellY +jj);
					}
				}
			}
			else
			{
				ROS_INFO("NO NEIGHBOURS");
			}
		}
	}
	return neighbourCellsFree;
}





bool Planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{

	if (isCellInMap(goalCellX, goalCellY) && goalCellValid(goalCellX, goalCellY)) 
	{
			findNeighbourValid(currCellX, currCellY);
	}
	else
	{
		ROS_INFO("THE GOAL CELL IS NOT INSIDE THE MAP OR IS AN OBSTACLE!");
	}
		
}

//lambanei to periexomeno tou keliou, 0 an einai eleuthero to keli, 100 an einai empodio kai -1 agnwsto
int Planner::getCellIndex(int ii, int jj) 
{
	int cellIndex = 0;
	return cellIndex;
}

//upologizetai to h(x) to opoio einai iso me thn apostash tou trexontos keliou apo ton teliko stoxo (gia oxi diagwnia)
int Planner::calculateHScore(int currCell, int goalCell) 
{
	int hScore = abs(currCellX - goalCellX) + abs(currCellY - goalCellY);
	return hScore;
}

//arxikopoiw to gScore se enan pinaka
std::vector <int> Planner::initializePlanner (int currCellX, int currCellY, int goalCellX, int goalCellY)
{
	float gScore[mapSize];
	
	for (unsigned ii = 0; ii < mapSize; ii ++) 
	{
		gScore[ii] = mapSize + 1;
	}

}




std::vector <int> Planner::path (int currCellX, int currCellY, int goalCellX, int goalCellY, float gScore[])
{
	std::vector <cell> OpenList;
	std::vector <cell> ClosedList;
	std::vector <int> bestPath;
	int possiblePath;
	
	cell C;
	C.currCellX = currCellX;
	C.currCellY = currCellY;
	OpenList.push_back(C);
		
	gScore[currCell] = 0; //gia to keli sto opoio vriskomaste
	float hScore[currCell] = calculateHScore(currCell, goalCell);
	float fScore[currCell] = gScore[currCell] + hScore[currCell];
		
	while (!OpenList.empty() && (gScore[goalCell] == mapSize + 1)) {
		
		std::vector <int> neighbourCell;
		neighbourCell = findNeighbourValid(currCellX, currCellY);
		for (unsigned int ii = 0; ii < value; ii++)
		{
			if (gScore[neighbourCell[ii]] == mapSize + 1)
			{
				ROS_INFO("DEN TO EXEI EPISKEFTHEI AKOMA");
				gScore[neighbourCell[ii]] = gScore[currCell] + 1;
				hScore[neighbourCell[ii]] = calculateHScore(neighbourCell[ii], goalCell);
				fScore[neighbourCell[ii]] = gScore[neighbourCell[ii]] + hScore[neighbourCell[ii]];
				addToOpenList(neighbourCellX, neighbourCellY, fScore);
			}
			gScore[currCell] = gScore[neighbourCell[ii]];
			
			
			
			//upologizei to kalutero monopati
			if (ii > 1) 
			{
				if (fScore[neighbourCell[ii]] < fScore[neighbourCell[ii - 1]]) {
					possiblePath = neighbourCell[ii];
				}
				else {
					possiblePath = neighbourCell[ii-1];
				}
			}
			else {
				possiblePath = neighbourCell[ii];
			}	
			
		}
		bestPath = path(possiblePath, gScore);
		C.currCell = possiblePath;
		OpenList.remove(C);
		ClosedList.push_back(C);
		
	}
	
	
}

//prosthetw sth lista to geitoniko keli
std::vector <cell> Planner::addToOpenList(int neighbourCellX, int neighbourCellY, float fScore)
{
	std::vector <cell> OpenList;
	cell C;
	C.currCellX = neighbourCellX;
	C.currCellY = neighbourCellY;
	C.fScore = fScore;
	OpenList.push_back(C);
}	


























