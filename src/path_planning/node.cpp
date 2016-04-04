#include "path_planning/node.h"

Node::Node()
{
	
}


void Node::uniforms()
{
	int curr_map_x = robot_perception.worldToMap(robot_perception.currentXPosition());
	int curr_map_y = robot_perception.worldToMap(robot_perception.currentYPosition());
	int goal_map_x = robot_perception.worldToMap(robot_perception.goalXPosition());
	int goal_map_y = robot_perception.worldToMap(robot_perception.goalYPosition());
	
	_step = 20;

	for (unsigned ii = 0; ii < robot_perception.getMapWidth(); ii = ii + _step)
	{
		for (unsigned jj = 0; jj < robot_perception.getMapHeight(); jj = jj + _step)
		{
		}
	}


	std::vector <node> neighbour_cell; //uniforms
	neighbour_cell.clear();
	node N; // gia uniforms

	int step = 20;
	int ext_step = 2 * step;
	for (unsigned ii = 0; ii < robot_perception.getMapWidth(); ii = ii + step)
	{
		for (unsigned jj = 0; jj < robot_perception.getMapHeight(); jj = jj + step)
		{
			if (robot_perception.rightCell(ii , jj))
			{
				N.x = ii;
				N.y = jj;
				
				neighbour_cell.push_back(N);
			}
		}
	}
	
	int curr_counter = 0;
	int goal_counter = 0;
	for (unsigned int zz = 0; zz < neighbour_cell.size(); zz ++)
	{
		if (!(curr_map_x == neighbour_cell[zz].x) && !(curr_map_y == neighbour_cell[zz].y))
		{
			curr_counter ++ ;
		}
		if (!(goal_map_x == neighbour_cell[zz].x) && !(goal_map_y == neighbour_cell[zz].y))
		{
			goal_counter ++ ;
		}
	}
	
	if (curr_counter == neighbour_cell.size())
	{
		N.x = curr_map_x;
		N.y = curr_map_y;
		//~ N.brushfire = _brushfire[curr_map_x][curr_map_y];
	}
	if (goal_counter == neighbour_cell.size())
	{
		N.x = goal_map_x;
		N.y = goal_map_y;
		//~ N.brushfire = _brushfire[goal_map_x][goal_map_y];
	}
}
