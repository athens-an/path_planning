#include "path_planning/particle.h"

Particle::Particle() 
{
	
}

Particle::Particle(unsigned int width, unsigned int height, int* data, std::vector<float> ranges, float resolution) 
{	
	//~ _x = std::rand() % (width) * resolution;
	//~ _y = std::rand() % (height) * resolution;
	 //~ 
	//~ while ((data[(int)(_x / resolution)][(int)(_y / resolution)] == - 1)
		//~ || (data[(int)(_x / resolution)][(int)(_y / resolution)] > 50))
	//~ {
		//~ _x = std::rand() % (width) * resolution;
		//~ _y = std::rand() % (height) * resolution;
	//~ }
//~ 
	//~ _theta = static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 2 * PI);
	
	_dx = _dy = _dtheta = _weight = 0;		
	_particle_ranges = new float[ranges.size()];	
	
	_x = 6;
	_y = 4;
	_theta = 0;
}

void Particle::move()
{
    _x += _dx;
    _y += _dy;
    _theta += _dtheta;

    _dx = _dy = _dtheta = 0;
}

void Particle::getRanges(float angle, unsigned int width, unsigned int height, int* data, float resolution, int ii)
{
	int distance = 1;
	float new_x = _x / resolution + distance * cos(angle);
	float new_y = _y / resolution + distance * sin(angle);
	float b = new_y * width + new_x;
	while (((int)new_x < width && (int)new_y < height) && ((int)new_x >= 0 && (int)new_y >= 0))
	{
		//~ ROS_INFO_STREAM("new_x = " << " " << (int)(new_x) << " " << "new_y = " << " " << (int)(new_y));
		if ((data[(int)(b)] > 50) ||  (data[(int)(b)] == -1))
		{
			_particle_ranges[ii] = (distance * resolution);
			return;
		}
		else
		{
			distance ++;
			new_x = _x / resolution + distance * cos(angle);
			new_y = _y / resolution + distance * sin(angle);
		}
	}
	
	_particle_ranges[ii] = ((distance - 1) * resolution);
}

float Particle::sense(std::vector<std::vector<float> > rfid_pose)
{
	float w = 0;
	float rfid_theta;
	float particle_theta = _theta;
	
	for (unsigned int ii = 0; ii < rfid_pose.size(); ii ++)
	{
		rfid_theta = atan2(rfid_pose[ii][1] - _y, rfid_pose[ii][0] - _x);
		float d_th = rfid_theta - particle_theta;
		if (cos(d_th) > cos(30 * PI / 180.0))
		{
			w = 1.0;
		}
		else
		{
			w = - 1.0;
		}
	}
	
	return w;
}

void Particle::setParticleWeight(unsigned int width, unsigned int height, int* data, float resolution, std::vector<float> ranges,
									float max_range, float increment, float angle_min, std::vector<std::vector<float> > rfid_pose)
{
	_weight = 0;
	float tag_w = sense(rfid_pose);
	
	std::vector<float> distances;
	float sum = 0;
		
	for (unsigned int ii = 0; ii < ranges.size(); ii ++)
	{
		getRanges(ii * increment + angle_min + _theta, width, height, data, resolution, ii);
	}

	for (unsigned int ii = 0; ii < ranges.size(); ii ++)
	{
		if (_particle_ranges[ii] / resolution <= 1)
		{
			_weight = 0;
			return;
		}
		else
		{	
			if (_particle_ranges[ii] > max_range)
			{
				_particle_ranges[ii] = max_range;
			}
			
			distances.push_back(fabs(ranges[ii]-_particle_ranges[ii]));
			sum += distances[ii];
		}
		//~ ROS_INFO_STREAM("distances : " << distances[i]);
	}
	
	float _sum_w = pow(1 / (sum + 1), 0.7);
	if(tag_w < 0.0)
	{
		_weight = _sum_w * 0.0001;
	}
	else
		_weight = _sum_w;
}

void Particle::calculateMotion(float previous_linear, float previous_angular, ros::Duration dt, float a1, float a2)
{
	//~ if (previous_angular == 0)
	//~ {
	  //~ _dx += (previous_linear * dt.toSec() * cosf(_theta));
	  //~ _dy += (previous_linear * dt.toSec() * sinf(_theta));
	//~ }
	//~ else
	//~ {
	  //~ _dx += (- previous_linear / previous_angular * sinf(_theta)
		//~ + previous_linear / previous_angular * 
		//~ sinf(_theta + dt.toSec() * previous_angular));
	  //~ 
	  //~ _dy -= (- previous_linear / previous_angular * cosf(_theta)
		//~ + previous_linear / previous_angular * 
		//~ cosf(_theta + dt.toSec() * previous_angular));
	//~ }
	
	_linear = previous_linear + noise(a1 * fabs(previous_linear) + a2 * fabs(previous_angular));
	_angular = previous_angular + noise(a2 * fabs(previous_linear) + a1 * fabs(previous_angular));
	
	if (_angular == 0)
	{
		_dx += (_linear * dt.toSec() * cosf(_theta));
		_dy += (_linear * dt.toSec() * sinf(_theta));
	}
	else
	{
		_dx += (- _linear / _angular * sinf(_theta) + _linear / _angular * sinf(_theta + dt.toSec() * _angular));
	  
		_dy -= (- _linear / _angular * cosf(_theta)	+ _linear / _angular * cosf(_theta + dt.toSec() * _angular));
	}
	
	_dtheta += _angular * dt.toSec() + noise(a1 * fabs(previous_linear) + a2 * fabs(previous_angular)) * dt.toSec();
}

float Particle::noise(float deviation) 
{
	float sum = 0;
	for ( unsigned int ii = 0; ii < 12; ii ++)
	{
		sum += - deviation + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2 * deviation)));
	}
	
	return 0.5 * sum;
}

float Particle::getX()
{
	return _x;
}

float Particle::getY() 
{
	return _y;
}

float Particle::getTheta() 
{
	return _theta;
}

float Particle::getWeight()
{
	return _weight;
}



