#include "path_planning/node.h"

Node::Node()
{
	_graph_pub = _node.advertise<visualization_msgs::Marker>("visualization_graph", 1);
}

void Node::uniforms(int width, int height, float resolution, int map_size, 
					int curr_map_x, int curr_map_y, int goal_map_x, int goal_map_y)
{
	_step = 20;
	
	_neighbour_cell_test.clear();
	node N; // gia uniforms (test)
	 
	
	int counter = 0;
	int curr_counter = 0;
	int goal_counter = 0;
	
	for (unsigned ii = 0; ii < width; ii = ii + _step)
	{
		for (unsigned jj = 0; jj < height; jj = jj + _step)
		{
			if (robot_perception.rightCell(ii, jj))
			{
				N.x = ii;
				N.y = jj;
				N.node_counter = counter;
				_neighbour_cell_test.push_back(N);
				
				counter ++ ;
			}
		}
	}
	
	int size = _neighbour_cell_test.size();
	for (unsigned int zz = 0; zz < _neighbour_cell_test.size(); zz ++)
	{
		if (!(curr_map_x == _neighbour_cell_test[zz].x && curr_map_y == _neighbour_cell_test[zz].y))
		{
			curr_counter ++ ;
		}
		if (!(goal_map_x == _neighbour_cell_test[zz].x && goal_map_y == _neighbour_cell_test[zz].y))
		{
			goal_counter ++ ;
		}
	}
	
	if (curr_counter == size)
	{
		N.x = curr_map_x;
		N.y = curr_map_y;
		N.node_counter = counter;
		counter ++ ;
		_neighbour_cell_test.push_back(N);
	}
	
	if (goal_counter == size)
	{
		N.x = goal_map_x;
		N.y = goal_map_y;
		N.node_counter = counter;
		counter ++ ;
		_neighbour_cell_test.push_back(N);
	}
	
	
	createGraph(map_size, resolution);
	
	//~ graph_obj.visualGraph(_neighbour_cell.size());
	
}

void Node::createGraph(int map_size, float resolution)
{
	_neighbour_cell.clear();
	node M;
	
	M.connections = new int *[_neighbour_cell_test.size()];
	M.distance = new float *[_neighbour_cell_test.size()];
	for (unsigned int ii = 0; ii < _neighbour_cell_test.size(); ii ++) 
	{
		M.connections[ii] = new int [_neighbour_cell_test.size()];
		M.distance[ii] = new float [_neighbour_cell_test.size()];
		for (unsigned int jj = 0; jj < _neighbour_cell_test.size(); jj ++) 
		{
			M.connections[ii][jj] = 0;
			M.distance[ii][jj] = map_size + 1;
		}
	}
	
	
	for (unsigned ii = 0; ii < _neighbour_cell_test.size(); ii ++)
	{
		M.x = _neighbour_cell_test[ii].x;
		M.y = _neighbour_cell_test[ii].y;
		M.node_counter = ii;
		for (unsigned jj = 0; jj < _neighbour_cell_test.size(); jj ++)
		{
			if (ii != jj)
			{
				
				if ((_neighbour_cell_test[ii].x <= _neighbour_cell_test[jj].x + _step && _neighbour_cell_test[ii].y == _neighbour_cell_test[jj].y))
				{
					if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y, 	
														_neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					{
						M.connections[ii][jj] = 1;
						M.distance[ii][jj] = abs(_neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x);
					}
				}
				
				if ((_neighbour_cell_test[ii].x <= _neighbour_cell_test[jj].x + _step && _neighbour_cell_test[ii].y <= _neighbour_cell_test[jj].y + _step))
				{
					if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y, 
														_neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					{
						M.connections[ii][jj] = 1;
						M.distance[ii][jj] = abs(_neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x);
					}
				}
				
				if ((_neighbour_cell_test[ii].x == _neighbour_cell_test[jj].x && _neighbour_cell_test[ii].y <= _neighbour_cell_test[jj].y + _step))
				{
					if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y,
														_neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					{
						M.connections[ii][jj] = 1;
						M.distance[ii][jj] = abs(_neighbour_cell_test[ii].y - _neighbour_cell_test[jj].y);
					}
				}
				if ((_neighbour_cell_test[ii].x <= _neighbour_cell_test[jj].x - _step && _neighbour_cell_test[ii].y <= _neighbour_cell_test[jj].y + _step))
				{
					if (obstacleSearch(_neighbour_cell_test[ii].x, _neighbour_cell_test[ii].y, 
														_neighbour_cell_test[jj].x, _neighbour_cell_test[jj].y))
					{
						M.connections[ii][jj] = 1;
						M.distance[ii][jj] = abs(_neighbour_cell_test[ii].x - _neighbour_cell_test[jj].x);
					}
				}
			
			}
		}
		_neighbour_cell.push_back(M);
	}
	
	visualGraph(_neighbour_cell.size(), resolution);
	
}

bool Node::obstacleSearch(int x1, int y1, int x2, int y2)
{
	int counter = 0;
	bool obst_flag = false;
	
	int x = abs(x1 - x2);
	int y = abs(y1 - y2);
	int step;
	
	if (x != 0)
	{
		step = x;
	}
	else
	{
		step = y;
	}
	
	//~ ROS_INFO_STREAM("5");
	for (unsigned int ii = 0; ii < step; ii ++)
	{
		if ((x1 == x2 + step && y1 == y2))
		{
			if (robot_perception.rightCell(x2 + ii, y2))
			{
				counter ++ ;
			}
		}
		if ((x1 == x2 + step && y1 == y2 + step))
		{
			if (robot_perception.rightCell(x2 + ii, y2 + ii))
			{
				counter ++ ;
			}
		}
		if ((x1 == x2 && y1 == y2 + step))
		{
			if (robot_perception.rightCell(x2, y2 + ii))
			{
				counter ++ ;
			}
		}
		if ((x1 == x2 - step && y1 == y2 + step))
		{
			if (robot_perception.rightCell(x2 - ii, y2 + ii))
			{
				counter ++ ;
			}
		}
	}
	
	if (counter == step)
	{
		//~ ROS_INFO_STREAM("FFFFFF");
		obst_flag = true;
	}
	
	return obst_flag;
	
}

int Node::getNeighbourCellSize()
{
	return _neighbour_cell.size();
}

int Node::getNeighbourCellX(int ii)
{
	return _neighbour_cell[ii].x;
}

int Node::getNeighbourCellY(int ii)
{
	return _neighbour_cell[ii].y;
}

int Node::getNeighbourCellNodeCounter(int curr_map_x, int curr_map_y)
{
	int curr_node;
	for (unsigned int ii = 0; ii < getNeighbourCellSize(); ii ++)
	{
		if ((curr_map_x == getNeighbourCellX(ii)) && (curr_map_y == getNeighbourCellY(ii)))
		{
			curr_node = _neighbour_cell[ii].node_counter;
		}
	}
	
	return curr_node;
}

int Node::getStep()
{
	return _step;
}

void Node::visualGraph(int size, float resolution)
{
	// Visualize the nodes
    visualization_msgs::Marker marker, line_strip;
	geometry_msgs::Point p;
	
	marker.header.frame_id = line_strip.header.frame_id = "map";
	marker.header.stamp = line_strip.header.stamp = ros::Time::now();
	
	marker.type = visualization_msgs::Marker::CUBE_LIST;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	
	marker.action = line_strip.action = visualization_msgs::Marker::ADD;
	
	marker.id = 0;
	line_strip.id = 1;
	
	marker.ns = line_strip.ns = "path_planning";
	
	
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = line_strip.pose.orientation.w = 1.0;
  
	marker.scale.x = 0.15;
	marker.scale.y = 0.15;
	marker.scale.z = 0.00;
	
	line_strip.scale.x = 0.05;
	
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;
	marker.color.b = 0.0;
	
	line_strip.color.a = 0.0;
	line_strip.color.r = 0.0;
	line_strip.color.g = 0.0;
	line_strip.color.b = 1.0;
	
	
	
	for (unsigned int ii = 0; ii < size; ii ++)
	{
		p.x = _neighbour_cell[ii].x * resolution;
		p.y = _neighbour_cell[ii].y * resolution;
		p.z = 0;
		marker.points.push_back(p);		
	}
	

	
	marker.lifetime = ros::Duration();
	
	_graph_pub.publish(marker);
	_graph_pub.publish(line_strip);
    
}

bool Node::getObstacleSearch(int ii, int jj)
{
	if (_neighbour_cell[ii].connections[ii][jj] == 1)
	{
		return true;
	}
	else
	{
		return false;
	}
	
}

float Node::getMinDistance(int ii)
{
	float infinity = std::numeric_limits<float>::infinity();
	float min = infinity;
	for (unsigned jj = 0; jj < _neighbour_cell.size(); jj ++)
	{
		if (min > _neighbour_cell[ii].distance[ii][jj])
		{			
			min = _neighbour_cell[ii].distance[ii][jj];
		}
		if (min > _neighbour_cell[jj].distance[jj][ii])
		{
			min = _neighbour_cell[jj].distance[jj][ii];
		}
	}
	
	return min;
}

//ean einai auto to shmeio tou xarth sto grafo
bool Node::getPixelGraph(int x, int y)
{
	bool flag = false;
	for (unsigned int ii = 0; ii < _neighbour_cell.size(); ii ++)
	{
		if ((x == _neighbour_cell[ii].x && y == _neighbour_cell[ii].y))
		{
			flag = true;
		}
	}
	
	return flag;
}




