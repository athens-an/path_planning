#include "path_planning/path_planning.h"


Planner::Planner()
{
	_goal_counter = 0;
	_service2 = _node.advertiseService("start", &Planner::start, this);
	_service1 = _node.advertiseService("goal", &Planner::goal, this);
	
	if(!_node.getParam("duration", _duration))
    {
		ROS_ERROR("Duration param does not exist");
	}
	
	if(!_node.getParam("distance_limit", _distance_limit))
    {
		ROS_ERROR("Distance_limit param does not exist");
	}
	
	if(!_node.getParam("yaw_limit", _yaw_limit))
    {
		ROS_ERROR("Yaw_limit param does not exist");
	}
	
	if(!_node.getParam("brushfire_const", _brushfire_const))
    {
		ROS_ERROR("Brushfire_const param does not exist");
	}
	
	if(!_node.getParam("brushfire_limit", _brushfire_limit))
    {
		ROS_ERROR("Brushfire_limit param does not exist");
	}
	
	_path_pub = _node.advertise<nav_msgs::Path>("/move_base_simple/path", 10);
	_marker_pub = _node.advertise<visualization_msgs::Marker>("visualization_marker", 1);
	_vel_pub = _node.advertise<geometry_msgs::Twist>("/robot0/cmd_vel", 1);
	
	//transform a point once every second
	_position_timer = _node.createTimer(ros::Duration(_duration), &Planner::currentPosition, this);
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
	
	// gia enan stoxo kathe fora
	//~ // ROS_INFO_STREAM("goal");
	//~ // 
	//~ // if (random())
	//~ // {
		//~ // res.success = true;
	//~ // }
	//~ // 
	//~ // else
	//~ // {
		//~ // ROS_INFO_STREAM("WRONG GOAL");
		//~ // res.success = false;
		//~ // return 1;
	//~ // }
 //~ 
	//~ std::vector <cell> best_path;
	//~ best_path.clear();
	//~ 
	//~ // _goal_cell_x = req.goal_cell_x;
	//~ // _goal_cell_y = req.goal_cell_y;
	//~ 
	//~ int curr_map_x = robot_perception.worldToMap(_curr_cell_x);
	//~ int curr_map_y = robot_perception.worldToMap(_curr_cell_y);
	//~ int goal_map_x = robot_perception.worldToMap(robot_perception.getGoalXPosition());
	//~ int goal_map_y = robot_perception.worldToMap(robot_perception.getGoalYPosition());
	//~ 
	//~ ROS_INFO_STREAM("GOAL X " << robot_perception.getGoalXPosition() << " GOAL Y " << robot_perception.getGoalYPosition());
	// _goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
	// _goal_map_y = worldToMap(_goal_cell_y);
	//~ 
	//~ if (robot_perception.rightCell(goal_map_x, goal_map_y))
	//~ {
		//~ best_path = path(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
		//~ ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y 
							//~ << " and a goal: " << robot_perception.getGoalXPosition() << " " << robot_perception.getGoalYPosition());
			//~ 
		//~ nav_msgs::Path plan;
		//~ geometry_msgs::PoseStamped pose;
 //~ 
		//~ plan.header.frame_id = "/map";
				//~ 
		//~ for (unsigned int ii = 0; ii < best_path.size(); ii ++)
		//~ {
//~ 
			//~ // geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Point
			//~ pose.pose.position.x = best_path[ii].x * robot_perception.getMapResolution();
			//~ pose.pose.position.y = best_path[ii].y * robot_perception.getMapResolution();
			//~ pose.pose.position.z = 0.0;
//~ 
			//~ // geometry_msgs/PoseStamped[] -> geometry_msgs/Pose -> geometry_msgs/Quaternion
			//~ pose.pose.orientation.x = 0.0;
			//~ pose.pose.orientation.y = 0.0;
			//~ pose.pose.orientation.z = 0.0;
			//~ pose.pose.orientation.w = 1.0;
//~ 
			//~ plan.poses.push_back(pose);
				//~ 
		//~ }
		//~ _path_pub.publish(plan);
		//~ res.success = true;
	//~ }
	//~ else
	//~ {
		//~ ROS_INFO_STREAM("WRONG GOAL");
		//~ res.success = false;
		//~ return 1;
	//~ }
	//~ 
	//~ return true;
	
	
	
	// gia synexomenous stoxous
	std::vector <cell> best_path;
	best_path.clear();
	
	YAML::Node baseNode = YAML::LoadFile("/home/athina/catkin_ws/src/path_planning/cfg/goals.yaml");
	if (baseNode.IsNull()) {
		ROS_INFO_STREAM("B");
		return false;
	}
	
	YAML::Node goalNode = baseNode["goal"];
	
	_goal_cell_x = goalNode["x"][_goal_counter].as<int>();
	_goal_cell_y = goalNode["y"][_goal_counter].as<int>();
	
	ROS_INFO_STREAM("GOAL X: " << _goal_cell_x << " GOAL Y: " << _goal_cell_y);
	
	int curr_node;

	int curr_map_x = robot_perception.worldToMap(_curr_cell_x);
	int curr_map_y = robot_perception.worldToMap(_curr_cell_y);
	int goal_map_x = robot_perception.worldToMap(_goal_cell_x);
	int goal_map_y = robot_perception.worldToMap(_goal_cell_y);
	
	if (robot_perception.rightCell(goal_map_x, goal_map_y))
	{
		best_path = path(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
		ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y 
							<< " and a goal: " << _goal_cell_x << " " << _goal_cell_y);
		
		if (_final_goal)
		{
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
				curr_node = node_obj.getNeighbourCellNodeCounter(best_path[ii].x, best_path[ii].y);
				ROS_INFO_STREAM("POSE_X " << pose.pose.position.x << " POSE_Y " << pose.pose.position.y << " NODE " << curr_node);
					
			}
			_path_pub.publish(plan);
			res.success = true;
		}
		else
		{
			
			ROS_INFO_STREAM("DEN MPOREI NA PAEI " << _goal_counter);
			res.success = false;
			return 1;
		}
	}
	else
	{
		ROS_INFO_STREAM("WRONG GOAL");
		_goal_counter ++;
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
	
	int curr_map_x = robot_perception.worldToMap(_curr_cell_x);
	int curr_map_y = robot_perception.worldToMap(_curr_cell_y);
	int goal_cell_x = robot_perception.getGoalXPosition();
	int goal_cell_y = robot_perception.getGoalYPosition();
	
	goal_cell_x = rand() % w;
	goal_cell_y = rand() % h;
	
	// ROS_INFO_STREAM("Goal cell " << _goal_cell_x << " " << _goal_cell_y);
	int goal_map_x = robot_perception.worldToMap(goal_cell_x); // map(pixel)
	int goal_map_y = robot_perception.worldToMap(goal_cell_y);
	
	while (!robot_perception.rightCell(goal_map_x, goal_map_y))
	{
		goal_cell_x = rand() % w;
		goal_cell_y = rand() % h;
	
		
		// ROS_INFO_STREAM("Goal cell " << _goal_cell_x << " " << _goal_cell_y);
		// _goal_map_x = worldToMap(_goal_cell_x); // map(pixel)
		// _goal_map_y = worldToMap(_goal_cell_y);
		
		int goal_map_x = robot_perception.worldToMap(goal_cell_x); // map(pixel)
		int goal_map_y = robot_perception.worldToMap(goal_cell_y);
		
	}
	
	// best_path = path(_curr_cell_x, _curr_cell_y, _goal_map_x, _goal_map_y);
	// ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y 
						// << " and a goal: " << _goal_cell_x << " " << _goal_cell_y);
						
	best_path = path(curr_map_x, curr_map_y, goal_map_x, goal_map_y);
	ROS_INFO_STREAM("Got a start: " << _curr_cell_x << " " << _curr_cell_y 
					<< " and a goal: " << robot_perception.getGoalXPosition() << " " << robot_perception.getGoalYPosition());
		
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

void Planner::currentPosition(const ros::TimerEvent& e)
{
	
	try
	{
		tf::StampedTransform transform;
		
		_listener.lookupTransform("/map", "nao_pose", ros::Time(0), transform);
		
		_curr_cell_x = transform.getOrigin()[0]; //transform.getOrigin().x()
		_curr_cell_y = transform.getOrigin()[1];
		_curr_cell_z = transform.getOrigin()[2];
		_yaw = tf::getYaw(transform.getRotation());

		
		ROS_INFO_STREAM("CURRENT POSITION: " << _curr_cell_x << " " 
							<< _curr_cell_y << " " << _yaw);
							
	}
	catch(tf::TransformException& ex){
		ROS_ERROR("%s",ex.what());
        ros::Duration(2.0).sleep();
	}
		
	
	
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
	node_obj.createNodes(robot_perception.getMapWidth(), robot_perception.getMapHeight(), 
						robot_perception.getMapResolution(), robot_perception.getMapSize(),
						curr_map_x, curr_map_y, goal_map_x, goal_map_y);
	

	int curr_node;
	int last_curr_node;
	curr_node = node_obj.getNeighbourCellNodeCounter(curr_map_x, curr_map_y);
	
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
	
	float nodes_distance = node_obj.getMinDistance(curr_node, last_curr_node);

	while (!open_list.empty() && !(curr_map_x == goal_map_x && curr_map_y == goal_map_y))
	{
		open_list.erase(open_list.begin() + counter);
		closed_list.push_back(C);
			
		float min = infinity;
		float new_g;
		
		last_curr_node = curr_node;
		for (unsigned int ii = nodes_distance; ii <= 3 * nodes_distance; ii = ii + nodes_distance) 
		{
			for (unsigned int jj = nodes_distance; jj <= 3 * nodes_distance; jj = jj + nodes_distance) 
			{	
				
				int mx = curr_map_x + ii - 2 * nodes_distance; //to x tou geitona
				int my = curr_map_y + jj - 2 * nodes_distance; //to y tou geitona
				int brushfire = robot_perception.getBrushfireCell(mx, my);
				
				//elegxw an anhkei ston grafo (eleythero) h oxi
				if (node_obj.getPixelGraph(mx, my) && brushfire > _brushfire_limit)
				{
					int neighbour_node = node_obj.getNeighbourCellNodeCounter(mx, my);
						
					if (g_score[mx][my] == robot_perception.getMapSize() + 1) //den exei epektathei akoma (den einai to trexon dhladh)
					{
						//elegxei tous geitones an enwnontai
						if (node_obj.getObstacleSearch(curr_node, neighbour_node))
						{
							//gia ta diagwnia kelia kelia
							if (!(mx == curr_map_x || my == curr_map_y))
							{
								g_score[mx][my] = g_score[curr_map_x][curr_map_y] + 1.4;
								//~ ROS_INFO_STREAM(" G1 " << g_score[mx][my] << " " << mx << " " << my);
							}
							else //gia ta panw-katw, deksia-aristera kelia
							{
								g_score[mx][my] = g_score[curr_map_x][curr_map_y] + 1;
								//~ ROS_INFO_STREAM(" G " << g_score[mx][my] << " " << mx << " " << my);
							}
							
							//~ ROS_INFO_STREAM("Not visited yet");
							h_score = calculateHScore(mx, my, goal_map_x, goal_map_y);
							f_score[mx][my] = g_score[mx][my] + h_score + _brushfire_const / brushfire;
							
							//pernaei sthn open list to geitoniko diathesimo keli
							N_C.x = mx;
							N_C.y = my;
							N_C.f_score = f_score[mx][my];
							N_C.cf_x = curr_map_x; //apo auto egine h epektash
							N_C.cf_y = curr_map_y;
							N_C.counter = c_f_counter;
							open_list.push_back(N_C);
						}
					}
					else //to exei ksanaepiskeuthei
					{
						if (!(ii - 2 * nodes_distance == 0) && (jj - 2 * nodes_distance) == 0)
						{
							for (unsigned int zz = 0; zz < open_list.size(); zz ++)
							{
								if (mx == open_list[zz].x && my == open_list[zz].y)
								{
									//gia ta diagwnia kelia kelia
									if (!(mx == curr_map_x || my == curr_map_y))
									{
										new_g = g_score[curr_map_x][curr_map_y] + 1.4;
									}
									else //gia ta panw-katw, deksia-aristera kelia
									{
										new_g = g_score[curr_map_x][curr_map_y] + 1;
									}
										
									if (new_g < g_score[mx][my])
									{
										g_score[mx][my] = new_g;
										f_score[mx][my] = g_score[mx][my] + calculateHScore(mx, my, goal_map_x, goal_map_y) 
															+ _brushfire_const / brushfire;
										open_list[zz].f_score = f_score[mx][my];
										open_list[zz].cf_x = curr_map_x;
										open_list[zz].cf_y = curr_map_y;
										open_list[zz].counter = c_f_counter;
									}
								}
							}
							//~ ROS_INFO_STREAM("Has visited");
						}
					}
					
				}
				else
				{
					//~ ROS_INFO_STREAM("DEN ANHKEI STON GRAFO");
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
				curr_node = node_obj.getNeighbourCellNodeCounter(curr_map_x, curr_map_y);
				counter = zz;
				//~ ROS_INFO_STREAM(" D " << robot_perception.getBrushfireCell(curr_map_x, curr_map_y) << " F " 
									//~ << min << " " << curr_map_x << " " << curr_map_y);
			}
		}
		
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
		
		nodes_distance = node_obj.getMinDistance(curr_node, last_curr_node);
		//~ ROS_INFO_STREAM("2");
	}
	
	C.x = goal_map_x;
	C.y = goal_map_y;
	C.counter = _came_from.size();
	 
	closed_list.push_back(C);
	//_came_from.push_back(C);
	
	
	if (finalGoal(_came_from, goal_map_x, goal_map_y))
	{
		best_path = reconstructPath(_came_from, goal_map_x, goal_map_y);
	}

	for (unsigned int ii = 0; ii < robot_perception.getMapWidth(); ii ++) 
	{
		delete [] g_score[ii];
		delete [] f_score[ii];
	}
	
	delete [] g_score;
	delete [] f_score;
	//~ delete [] _index;
	
	_goal_counter ++;
	
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
		ROS_INFO_STREAM("SIZE " << count);
		current_x = _came_from[count - 1].x;
		current_y = _came_from[count - 1].y;
		count = _came_from[count - 1].counter;
				
		ROS_INFO_STREAM("reConstructorPath " << current_x << " " << current_y << " " << count);
		B.x = current_x;
		B.y = current_y;
		best_path.push_back(B);
		
		if (counter1 % 1 == 0)
		{
			end_x = current_x;
			end_y = current_y;
			S.x = current_x;
			S.y = current_y;
			subobjective_path.push_back(S);
		}
		counter1 ++;
	}
	//~ ROS_INFO_STREAM("reConstr " << current_x << " " << current_y << " " << end_x << " " << end_y);

	if (!(end_x * robot_perception.getMapResolution() == _curr_cell_x) 
		|| !(end_y * robot_perception.getMapResolution() == _curr_cell_y))
	{
		S.x = _curr_cell_x / robot_perception.getMapResolution();
		S.y = _curr_cell_y / robot_perception.getMapResolution();
		subobjective_path.push_back(S);
		best_path.push_back(S);
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
		
		ROS_INFO_STREAM("targets " << i << " " << j << " F " << f_score[i][j] << " Brush " << robot_perception.getBrushfireCell(i, j));
		
	}

	
	_vel_timer = _node.createTimer(ros::Duration(_duration), &Planner::velocity, this);		
	
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
	
	
	ROS_INFO_STREAM("DOKIMI " << _dokimi);
	
	if (_dokimi > 0)
	{
		float current_x = _current_pose.x * robot_perception.getMapResolution();
		float current_y = _current_pose.y * robot_perception.getMapResolution();
		float sub_target_x = _sub_target.x * robot_perception.getMapResolution();
		float sub_target_y = _sub_target.y * robot_perception.getMapResolution();
		
		float dis = distance(_curr_cell_x, _curr_cell_y, sub_target_x, sub_target_y);
		float pi = 3.14159;
		
		_z = atan2(sub_target_y - current_y, sub_target_x - current_x) / 1.0;
		
		
		float yaw = _yaw;
		
		if (dis > 0)
		{
			if (!(yaw < _z + _yaw_limit && yaw > _z - _yaw_limit))
			{
				twist.angular.z = - yaw + atan2(sub_target_y - current_y, sub_target_x - current_x) / 1.0;
				if (twist.angular.z > - pi && twist.angular.z <= 0)
				{
					twist.angular.z = twist.angular.z / pi;
					
				}
				else if (twist.angular.z < - pi && twist.angular.z < 0)
				{
					twist.angular.z = (twist.angular.z + 2 * pi) / pi;
					
				}
				else if (twist.angular.z >= pi && twist.angular.z > 0)
				{
					twist.angular.z = (twist.angular.z - 2 * pi) / pi;
					
				}
				else if (twist.angular.z < pi && twist.angular.z >= 0)
				{
					twist.angular.z = twist.angular.z / pi;
					
				}
				else
				{
				}
				_vel_pub.publish(twist);
				test();
			}
			else
			{
				twist.linear.x = 0.05;
				_vel_pub.publish(twist);
				ROS_INFO_STREAM("5 ");
				//~ ROS_INFO_STREAM("X: " << _curr_cell_x << " Y: " << _curr_cell_y);
				ROS_INFO_STREAM("YAW: " << _yaw << " GOAL_YAW: " << _z);
				ROS_INFO_STREAM("TWIST: " << twist.angular.z);
				ROS_INFO_STREAM("CURR_X: " << current_x << " CURR_Y: " << current_y);
				ROS_INFO_STREAM("SUB_X: " << sub_target_x << " SUB_Y: " << sub_target_y);
				
				if (dis < _distance_limit)
				{
					_dokimi ++;
				}
				test();
			}
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

bool Planner::finalGoal(const std::vector <cell>& _came_from, 
							int goal_map_x, int goal_map_y)
{
	_final_goal = false;
	for (unsigned int ii = 0; ii < _came_from.size(); ii ++)
	{
		if (goal_map_x == _came_from[ii].x && goal_map_y == _came_from[ii].y)
		{
			_final_goal = true;
		}
	}
	
	return _final_goal;
}

