#include "path_planning/path_planning.h"


Planner::Planner()
{
	_service2 = _node.advertiseService("start", &Planner::start, this);
	_service1 = _node.advertiseService("goal", &Planner::goal, this);
	
	_path_pub = _node.advertise<nav_msgs::Path>("/move_base_simple/path", 10);
	_marker_pub = _node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	_vel_pub = _node.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
		
}

bool Planner::start(path_planning::startRequest &req, 
					path_planning::startResponse &res)
{
	ROS_INFO_STREAM("start");
	res.success = true;
	return true;
}


bool Planner::goal(path_planning::goalRequest &req, 
					path_planning::goalResponse &res)
{
	//~ ROS_INFO_STREAM("goal");
	//~ 
	//~ if (random())
	//~ {
		//~ res.success = true;
	//~ }
	//~ 
	//~ else
	//~ {
		//~ ROS_INFO_STREAM("WRONG GOAL");
		//~ res.success = false;
		//~ return 1;
	//~ }
 
	std::vector <cell> best_path;
	best_path.clear();
	
	//~ _goal_cell_x = req.goal_cell_x;
	//~ _goal_cell_y = req.goal_cell_y;
	
	int curr_map_x = robot_perception.worldToMap(robot_perception.currentXPosition());
	int curr_map_y = robot_perception.worldToMap(robot_perception.currentYPosition());
	int goal_map_x = robot_perception.worldToMap(robot_perception.goalXPosition());
	int goal_map_y = robot_perception.worldToMap(robot_perception.goalYPosition());
	
	ROS_INFO_STREAM("GOAL X " << robot_perception.goalXPosition() << " GOAL Y " << robot_perception.goalYPosition());
	//~ _goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
	//~ _goal_map_y = worldToMap(_goal_cell_y);
	
	if (robot_perception.rightCell(goal_map_x, goal_map_y))
	{
		best_path = path(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
		ROS_INFO_STREAM("Got a start: " << robot_perception.currentXPosition() << " " << robot_perception.currentYPosition() 
							<< " and a goal: " << robot_perception.goalXPosition() << " " << robot_perception.goalYPosition());
			
		nav_msgs::Path plan;
		geometry_msgs::PoseStamped pose;
 
		plan.header.frame_id = "/map";
				
		for (unsigned int ii = 0; ii < best_path.size(); ii ++)
		{

			// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Point
			pose.pose.position.x = best_path[ii].x * robot_perception.getMapResolution();
			pose.pose.position.y = best_path[ii].y * robot_perception.getMapResolution();
			pose.pose.position.z = 0.0;

			//~ // geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Quaternion
			pose.pose.orientation.x = 0.0;
			pose.pose.orientation.y = 0.0;
			pose.pose.orientation.z = 0.0;
			pose.pose.orientation.w = 1.0;

			plan.poses.push_back(pose);
				
		}
		_path_pub.publish(plan);
		res.success = true;
	}
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
	
	int w = robot_perception.getMapWidth() * robot_perception.getMapResolution();
	int h = robot_perception.getMapHeight() * robot_perception.getMapResolution();
	
	int curr_map_x = robot_perception.worldToMap(robot_perception.currentXPosition());
	int curr_map_y = robot_perception.worldToMap(robot_perception.currentYPosition());
	int goal_cell_x = robot_perception.goalXPosition();
	int goal_cell_y = robot_perception.goalYPosition();
	
	goal_cell_x = rand() % w;
	goal_cell_y = rand() % h;
	
	//~ ROS_INFO_STREAM("Goal cell " << _goal_cell_x << " " << _goal_cell_y);
	int goal_map_x = robot_perception.worldToMap(goal_cell_x); // map(pixel)
	int goal_map_y = robot_perception.worldToMap(goal_cell_y);
	
	while (!robot_perception.rightCell(goal_map_x, goal_map_y))
	{
		goal_cell_x = rand() % w;
		goal_cell_y = rand() % h;
	
		
		//~ ROS_INFO_STREAM("Goal cell " << _goal_cell_x << " " << _goal_cell_y);
		//~ _goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
		//~ _goal_map_y = worldToMap(_goal_cell_y);
		
		int goal_map_x = robot_perception.worldToMap(goal_cell_x); // map(pixel)
		int goal_map_y = robot_perception.worldToMap(goal_cell_y);
		
	}
	
	//~ best_path = path(_curr_cell_x, _curr_cell_y, _goal_map_x, _goal_map_y);
	//~ ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y 
						//~ << " and a goal: " << _goal_cell_x << " " << _goal_cell_y);
						
	best_path = path(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
	ROS_INFO_STREAM("Got a start: " << robot_perception.currentXPosition() << " " << robot_perception.currentYPosition() 
					<< " and a goal: " << robot_perception.goalXPosition() << " " << robot_perception.goalYPosition());
		
	nav_msgs::Path plan;
	geometry_msgs::PoseStamped pose;


	plan.header.frame_id = "/map";
		
		
	for (unsigned int ii = 0; ii < best_path.size(); ii ++)
	{

		// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Point
		pose.pose.position.x = best_path[ii].x * robot_perception.getMapResolution();
		pose.pose.position.y = best_path[ii].y * robot_perception.getMapResolution();
		pose.pose.position.z = 0.0;

		// geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Quaternion
		pose.pose.orientation.x = 0.0;
		pose.pose.orientation.y = 0.0;
		pose.pose.orientation.z = 0.0;
		pose.pose.orientation.w = 1.0;

		plan.poses.push_back(pose);
			
	}
	_path_pub.publish(plan);
	return true;
}

//calculate h(x), manhattan - euclidean distance
float Planner::calculateHScore (int curr_map_x, int curr_map_y, 
								int goal_map_x, int goal_map_y) 
{
	//~ float h_score = abs(curr_map_x - goal_map_x) + abs(curr_map_y - goal_map_y);
	float h_score = (goal_map_x - curr_map_x) * (goal_map_x - curr_map_x) + (goal_map_y - curr_map_y) * (goal_map_y - curr_map_y);
	h_score = sqrt(h_score);
	return h_score;
}

std::vector <cell> Planner::path (int curr_map_x, int curr_map_y, 
									int goal_map_x, int goal_map_y)
{
	std::vector <cell> open_list;
	std::vector <cell> closed_list;
	std::vector <cell> best_path;
	std::vector <cell> _came_from;
	
	open_list.clear();
	closed_list.clear();
	best_path.clear();
	_came_from.clear();
	
	cell C; // gia closed_list kai came_from
	cell N_C; // gia open_list
	cell C_F; // gia came_from
	
	
	float infinity = std::numeric_limits<float>::infinity();
		
	g_score = new float *[robot_perception.getMapWidth()];
	for (unsigned int ii = 0; ii < robot_perception.getMapWidth(); ii ++) 
	{
		g_score[ii] = new float [robot_perception.getMapHeight()];
		for (unsigned int jj = 0; jj < robot_perception.getMapHeight(); jj ++) 
		{
			g_score[ii][jj] = robot_perception.getMapSize() + 1;
		}
	}
		
	f_score = new float *[robot_perception.getMapWidth()];
	for (unsigned int ii = 0; ii < robot_perception.getMapWidth(); ii ++) 
	{
		f_score[ii] = new float [robot_perception.getMapHeight()];
		for (unsigned int jj = 0; jj < robot_perception.getMapHeight(); jj ++) 
		{
			f_score[ii][jj] = std::numeric_limits<float>::infinity();
		}
	}
	
	robot_perception.brushfire();
	
	//prosthetw thn trexousa thesh sthn open list
	C.x = curr_map_x;
	C.y = curr_map_y;
	open_list.push_back(C);
	
	
	int c_f_counter = 0; // gia na kserei apo poio keli egine h eksaplwsh
	int g_counter = 0; // counter gia ta g_score
	int counter = 0; // gia na kserei poio keli tha diagrapsei apo thn open list
		
	g_score[curr_map_x][curr_map_y] = 0; //gia to keli sto opoio vriskomaste
	float h_score = calculateHScore(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
	f_score[curr_map_x][curr_map_y] = g_score[curr_map_x][curr_map_y] + h_score;
	
	while (!open_list.empty() && !(curr_map_x == goal_map_x && curr_map_y == goal_map_y))
	{
		open_list.erase(open_list.begin() + counter);
		closed_list.push_back(C);
			
		float min = infinity;
		float new_g;
		
		for (unsigned int ii = 1; ii <= 3; ii ++) 
		{
			for (unsigned int jj = 1; jj <= 3; jj ++) 
			{				
				int mx = curr_map_x + ii - 2; //to x tou geitona
				int my = curr_map_y + jj - 2; //to y tou geitona
				
				//elegxei tous geitones
				if ((ii - 2) == 0 && (jj - 2) == 0)
				{
					//~ ROS_INFO_STREAM("No Neighbour");
				}				
				else
				{
					//elegxw an einai mesa ston xarth kai eleuthero h oxi
					if (robot_perception.rightCell(mx, my))
					{
						if (g_score[mx][my] == robot_perception.getMapSize() + 1) //den exei epektathei akoma
						{
							//gia ta diagwnia kelia kelia
							if (((mx - curr_map_x) == -1 && (my - curr_map_y) == -1)
								|| ((mx - curr_map_x) == -1 && (my - curr_map_y) == 1)
								|| ((mx - curr_map_x) == 1 && (my - curr_map_y) == -1)
								|| ((mx - curr_map_x) == 1 && (my - curr_map_y) == 1))
							{
								g_score[mx][my] = g_score[curr_map_x][curr_map_y] + 1.4;
								//~ ROS_INFO_STREAM(" G " << g_score[mx][my] << " " << mx << " " << my);
							}
							else //gia ta panw-katw, deksia-aristera kelia
							{
								g_score[mx][my] = g_score[curr_map_x][curr_map_y] + 1;
								//~ ROS_INFO_STREAM(" G " << g_score[mx][my] << " " << mx << " " << my);
							}
							
							//~ ROS_INFO_STREAM("Not visited yet");
							h_score = calculateHScore(mx, my, goal_map_x, goal_map_y);
							f_score[mx][my] = g_score[mx][my] + h_score + 800 / robot_perception.getBrushfireCell(mx, my);
							//~ f_score[mx][my] = f_score[mx][my] * f_score[mx][my];
							//~ ROS_INFO_STREAM(" D " << _brushfire[mx][my] << " F " << f_score[mx][my] << " " << mx << " " << my);
							
							//pernaei sthn open list to geitoniko diathesimo keli
							N_C.x = mx;
							N_C.y = my;
							N_C.f_score = f_score[mx][my];
							N_C.cf_x = curr_map_x; //apo auto egine h epektash
							N_C.cf_y = curr_map_y;
							N_C.counter = c_f_counter;
							open_list.push_back(N_C);
						}
						else //to exei ksanaepiskeuthei
						{
							for (unsigned int zz = 0; zz < open_list.size(); zz ++)
							{
								if (mx == open_list[zz].x && my == open_list[zz].y)
								{
									//gia ta diagwnia kelia kelia
									if ((mx - curr_map_x) == -1 && (my - curr_map_y) == -1
										|| (mx - curr_map_x) == -1 && (my - curr_map_y) == 1
										|| (mx - curr_map_x) == 1 && (my - curr_map_y) == -1
										|| (mx - curr_map_x) == 1 && (my - curr_map_y) == 1)
									{
										new_g = g_score[curr_map_x][curr_map_y] + 1.4;
									}
									else //gia ta panw-katw, deksia-aristera kelia
									{
										new_g = g_score[curr_map_x][curr_map_y] + 1;
									}
										
									if (new_g < g_score[mx][my])
									{
										//~ ROS_INFO_STREAM(" D " << _brushfire[mx][my]);
										g_score[mx][my] = new_g;
										f_score[mx][my] = g_score[mx][my] + calculateHScore(mx, my, goal_map_x, goal_map_y) 
															+ 800 / robot_perception.getBrushfireCell(mx, my);
										//~ f_score[mx][my] = f_score[mx][my] * f_score[mx][my];
										open_list[zz].f_score = f_score[mx][my];
										open_list[zz].cf_x = curr_map_x;
										open_list[zz].cf_y = curr_map_y;
										open_list[zz].counter = c_f_counter;
									}
								}
							}
							//~ ROS_INFO_STREAM("Has visited");
						}
						//~ ROS_INFO_STREAM(" X " << mx << " Y " << my << " FSCORE " << f_score[mx][my]);
					}
				}
			}
		}
		
		//upologizei to mikrotero f_score apo ta kelia pou exei episkeuthei mexri stigmhs
		for (unsigned int zz = 0; zz < open_list.size(); zz ++)
		{
			if (min > open_list[zz].f_score)
			{	
				min = open_list[zz].f_score;
				curr_map_x = open_list[zz].x;
				curr_map_y = open_list[zz].y;
				counter = zz;
				//~ ROS_INFO_STREAM(" D " << _brushfire[curr_map_x][curr_map_y] << " F " 
									//~ << min << " " << curr_map_x << " " << curr_map_y);
			}
		}
		//~ ROS_INFO_STREAM("MIN " << min);
		
		//~ ROS_INFO_STREAM("Next cell " << curr_map_x << " " << curr_map_y);
		C.x = curr_map_x;
		C.y = curr_map_y;
		
		C_F.x = curr_map_x;
		C_F.y = curr_map_y;
		C_F.cf_x = open_list[counter].cf_x;
		C_F.cf_y = open_list[counter].cf_y;
		C_F.f_score = open_list[counter].f_score;
		C_F.counter = open_list[counter].counter;
		c_f_counter ++ ;
		_came_from.push_back(C_F);
	}
	
	C.x = goal_map_x;
	C.y = goal_map_y;
	C.counter = _came_from.size();
	 
	
	closed_list.push_back(C);
	_came_from.push_back(C);
	
	
	best_path = reconstructPath(_came_from, goal_map_x, goal_map_y);
	

	for (unsigned int ii = 0; ii < robot_perception.getMapWidth(); ii ++) 
	{
		delete [] g_score[ii];
		delete [] f_score[ii];
	}
	delete [] g_score;
	delete [] f_score;
	//~ delete [] _index;
		

	return best_path;
}

std::vector <cell> Planner::reconstructPath (const std::vector <cell>& _came_from, 
												int goal_map_x, int goal_map_y)
{
	std::vector <cell> best_path;
	std::vector <cell> subobjective_path;
	
	best_path.clear();
	
	int count = _came_from.size();
	cell B;
	cell S;

	int sub_target_x = goal_map_x;
	int current_x = goal_map_x;
	int current_y = goal_map_y;
	int sub_target_y = goal_map_y;
		
	int counter1 = 0; // for sub_target
	_dokimi = 0;
	int end_x;
	int end_y;
	
	while (!(count == 0))
	{
		current_x = _came_from[count - 1].x;
		current_y = _came_from[count - 1].y;
		count = _came_from[count - 1].counter;
		
		//~ ROS_INFO_STREAM("reConstructorPath " << current_x << " " << current_y);
		B.x = current_x;
		B.y = current_y;
		best_path.push_back(B);
		
		if (counter1 % 20 == 0)
		{
			end_x = current_x;
			end_y = current_y;
			S.x = current_x;
			S.y = current_y;
			subobjective_path.push_back(S);
		}
		counter1 ++;
	}
	ROS_INFO_STREAM("reConstr " << current_x << " " << current_y << " " << end_x << " " << end_y);

	if (!(end_x * robot_perception.getMapResolution() == robot_perception.currentXPosition()) 
		|| !(end_y * robot_perception.getMapResolution() == robot_perception.currentXPosition()))
	{
		S.x = robot_perception.currentXPosition() / robot_perception.getMapResolution();
		S.y = robot_perception.currentXPosition() / robot_perception.getMapResolution();
		subobjective_path.push_back(S);
	}
	_counter = subobjective_path.size();
	
	_target_x = new float [_counter];
	_target_y = new float [_counter];
	for (unsigned int ii = 0; ii < _counter; ii ++) 
	{
		_target_x[ii] = subobjective_path[_counter - 1 - ii].x;
		_target_y[ii] = subobjective_path[_counter - 1 - ii].y;
		
		int i = _target_x[ii];
		int j = _target_y[ii];
		
		ROS_INFO_STREAM("targets " << i << " " << j << " F " << f_score[i][j]);
		
	}

	
	_vel_timer = _node.createTimer(ros::Duration(1.0), &Planner::velocity, this);		
	
	visual(subobjective_path);
	return best_path;
}

void Planner::visual(const std::vector <cell>& subobjective_path)
{
	 // Visualize the nodes
    visualization_msgs::Marker marker;
	geometry_msgs::Point p;
	
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time::now();
	
	marker.type = visualization_msgs::Marker::SPHERE_LIST;
	marker.action = visualization_msgs::Marker::ADD;
	
	marker.id = 0;
	marker.ns = "path_planning";
	
	for (unsigned int ii = 0; ii < subobjective_path.size(); ii ++)
	{
		p.x = subobjective_path[ii].x * robot_perception.getMapResolution();
		p.y = subobjective_path[ii].y * robot_perception.getMapResolution();
		p.z = 0;
		marker.points.push_back(p);	
	}
	
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
  
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.00;
	marker.color.a = 1.0;
	marker.color.r = 0.0;
	marker.color.g = 1.0;
	marker.color.b = 0.0;
	
	marker.lifetime = ros::Duration();
	
	_marker_pub.publish(marker);
    
}

void Planner::velocity(const ros::TimerEvent& event)
{
	
	geometry_msgs::Twist twist;
	
	twist.linear.x = 0.0;
	twist.linear.y = 0.0;
    twist.linear.z = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
	
	if (_dokimi > 0)
	{
		float current_x = _current_pose.x * robot_perception.getMapResolution();
		float current_y = _current_pose.y * robot_perception.getMapResolution();
		float sub_target_x = _sub_target.x * robot_perception.getMapResolution();
		float sub_target_y = _sub_target.y * robot_perception.getMapResolution();
		
		float dis = distance(current_x, current_y, sub_target_x, sub_target_y);
		float pi = 3.14159;
		
		_z = atan2(sub_target_y - current_y, sub_target_x - current_x)/1.0;
		
		float yaw = robot_perception.currentYaw();
		float q = yaw - _z;

		if (!((yaw < _z + 0.01) && (yaw > _z - 0.01)))
		{
			twist.angular.z = - yaw + atan2(sub_target_y - current_y, sub_target_x - current_x)/1.0;
			if (twist.angular.z > - pi && twist.angular.z < 0)
			{
				twist.angular.z = twist.angular.z / pi;
			}
			else if (twist.angular.z < - pi)
			{
				twist.angular.z = (twist.angular.z + 2 * pi) / pi;
			}
			else if (twist.angular.z > pi)
			{
				twist.angular.z = (twist.angular.z - 2 * pi) / pi;
			}
			else if (twist.angular.z < pi && twist.angular.z > 0)
			{
				twist.angular.z = twist.angular.z / pi;
			}
			else
			{
				ROS_INFO_STREAM(" ");
			}
			_vel_pub.publish(twist);
			test();
		}
		else
		{
			twist.linear.x = dis;
			_vel_pub.publish(twist);
			_dokimi ++;
			test();
		}
	}
	else
	{
		_dokimi ++;
		test();
	}
		
}

void Planner::test()
{
	_current_pose.x = _target_x[_dokimi - 1];
	_current_pose.y = _target_y[_dokimi - 1];
	
	_sub_target.x = _target_x[_dokimi];
	_sub_target.y = _target_y[_dokimi];
	
	if (_dokimi > 0)
	{
		_x = _target_x[_dokimi - 1];
		_y = _target_y[_dokimi - 1];
	}
	
	if ((_current_pose.x == _sub_target.x) && (_current_pose.y == _sub_target.y))
	{
		_dokimi = 0;
	}
}

float Planner::distance(float x1, float y1, float x2, float y2)
{
	float dis = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	dis = sqrt(dis);
	return dis;
}





