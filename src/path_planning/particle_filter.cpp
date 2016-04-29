#include "path_planning/particle_filter.h"

ParticleFilter::ParticleFilter() 
{
	_previous_linear = 0;
	_previous_angular = 0;
	_current_angular = 0;
	_current_linear = 0;
	_previous_time = ros::Time::now();
	
	_flag = false;
	_motion_flag = false;
	_particles_initialized = false;
	
	_particle_initialization_service = _node.advertiseService ("particles_initialization", &ParticleFilter::particlesInit, this);
	ROS_INFO("Service ready to initialize particles");
	
	if(!_node.getParam("/enable_visualization", _visualization_enabled))
    {
		ROS_ERROR("Enable_visualization param does not exist");
	}
	
	if(!_node.getParam("duration", _duration))
	{
		ROS_ERROR("Duration param does not exist");
	}
	
	if(!_node.getParam("a1", _noise_param1))
	{
		ROS_ERROR("Noise param a1 does not exist");
	}
	
	if(!_node.getParam("a2", _noise_param2))
	{
		ROS_ERROR("Noise param a2 does not exist");
	}
	
	_visualization_pub = _node.advertise<visualization_msgs::Marker>("visualization_marker", 0);
            
    _velocity_sub = _node.subscribe("/robot0/cmd_vel", 10, &ParticleFilter::velocityCallback, this);
    
    _timer = _node.createTimer(ros::Duration(_duration), &ParticleFilter::particlesCallback, this);
    
}

bool ParticleFilter::particlesInit(path_planning::particleInitSrv::Request& req,
									path_planning::particleInitSrv::Response& res)
{
	if(!_node.getParam("/particles_number", _particles_number))
	{
		ROS_ERROR("Particles number param does not exist");
	}
	
	for (unsigned int ii = 0; ii < _particles_number; ii ++)
	{
		
		Particle particle(robot_perception.getMapWidth(), robot_perception.getMapHeight(), robot_perception.getMapIndex(), 
							robot_perception.getLaserRanges(), robot_perception.getMapResolution());
		
		_particles.push_back(particle);
	
	}
	
	_particles_initialized = true;
	
	ROS_INFO_STREAM(_particles_number << " " << "particles initialized");
	if(_visualization_enabled)
	{
		visualize(robot_perception.getMapResolution());
	}
	
	res.success = true;
	return true;	
}

void ParticleFilter::particlesCallback(const ros::TimerEvent& event)
{
	if (_particles_initialized)
	{
		if (_previous_angular || _previous_linear)
		{
			_motion_flag = true;
		}
		
		_current_time = ros::Time::now();
		_dt = _current_time - _previous_time;
		
		for (unsigned int ii = 0; ii < _particles_number; ii ++)
		{
			_particles[ii].calculateMotion(_previous_linear, _previous_angular, _dt, _noise_param1, _noise_param2);
			_particles[ii].move();
			//~ ROS_INFO_STREAM("New x = " << _particles[ii].getX() << " new y = " << _particles[ii].getY());
			_previous_angular = _current_angular;
			_previous_linear = _current_linear;
			_previous_time = _current_time;
		}
		
		visualize(robot_perception.getMapResolution());
		
		for (unsigned int ii = 0; ii < _particles_number; ii ++)
		{
			_particles[ii].setParticleWeight(robot_perception.getMapWidth(), robot_perception.getMapHeight(), robot_perception.getMapIndex(),
												robot_perception.getMapResolution(), robot_perception.getLaserRanges(),
												robot_perception.getRangeMax(), robot_perception.getAngleIncrement(),
												robot_perception.getAngleMin(), robot_perception.getRfidPose());
		}
		
		if (_motion_flag)
		{
			resample();
		}
		
		_motion_flag = false;
		visualize(robot_perception.getMapResolution());
	}
}

void ParticleFilter::resample()
{
	float average = 0;
	float sum = 0;
	bool flag = false;
	
	for (unsigned int ii = 0; ii < _particles_number; ii ++) 
	{
		if (_particles[ii].getWeight() > 0.00001)
		{
			flag = true;
			break;
		}
	}
	
	for (unsigned int ii = 0; ii < _particles_number; ii ++) 
	{
		sum += _particles[ii].getWeight();
	}
	
	average = sum/ _particles_number;
	ROS_INFO_STREAM("average1 = " << average);
	
	if (flag == true)
	{
		ROS_INFO_STREAM("resample");		
		sum = 0;
		std::vector<Particle> new_particles;
		
		int index = std::rand() % ( _particles_number );
		float beta = 0.0;
		float max_weight = _particles[0].getWeight();
		
		for (unsigned int ii = 0; ii < _particles_number; ii ++) 
		{
			if (_particles[ii].getWeight() > max_weight)
				max_weight = _particles[ii].getWeight();
		}
		
		for (unsigned int ii = 0; ii < _particles_number; ii ++) 
		{
			beta += static_cast <float> (rand()) / static_cast <float> (RAND_MAX / 2 * max_weight); // parentheseis sto rand max
			
			while (beta > _particles[index].getWeight())
			{
				beta -= _particles[index].getWeight();
				index = (index + 1) % _particles_number;
			}
			new_particles.push_back(_particles[index]);
		}
	
		_particles = new_particles;
		
		for (unsigned int ii = 0; ii < _particles_number; ii ++ ) 
		{
			sum += _particles[ii].getWeight();
		}
		
		average = sum / _particles_number;
		ROS_INFO_STREAM("average2 = " << average);
	}
}

void ParticleFilter::velocityCallback(geometry_msgs::Twist twist)
{
	//~ ROS_INFO_STREAM("VelocityCallback");
	_current_linear = twist.linear.x;
	_current_angular = twist.angular.z;

	if (_particles_initialized)
	{
		if ((_current_angular != _previous_angular) || (_current_linear != _previous_linear))
		{
			if (_flag == false)
			{
				_previous_time = ros::Time::now();
				_previous_angular = _current_angular;
				_previous_linear = _current_linear;
				_flag = true;
			}
			else
			{
				_motion_flag = true;
				_current_time = ros::Time::now();
				_dt = _current_time - _previous_time;
				
				for (int ii = 0; ii < _particles_number; ii ++)
				{
					_particles[ii].calculateMotion(_previous_linear, _previous_angular, _dt, _noise_param1, _noise_param2);
				}
				
				_previous_angular = _current_angular;
				_previous_linear = _current_linear;
				_previous_time = _current_time;
			}
		}
	}
}

void ParticleFilter::visualize(float resolution)
{
	visualization_msgs::Marker m, m1;
	
	m.header.frame_id = "map";
    m.header.stamp = ros::Time();
    m.type = visualization_msgs::Marker::SPHERE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.id = 0;
    m.ns = "Particles";
    m.scale.x = 0.1;
    m.scale.y = 0.1;
    m.scale.z = 0.1;
    m.color.a = 1.0;
    m.color.r = 1.0;
    m.color.g = 0.0;
    m.color.b = 0.0;
    
    for(unsigned int ii = 0; ii < _particles_number; ii ++)
    {
      geometry_msgs::Point p;
      p.x = _particles[ii].getX();
      p.y = _particles[ii].getY();
      m.points.push_back(p);
    }
	
	_visualization_pub.publish(m);
	 
	m1.header.frame_id = "map";
    m1.header.stamp = ros::Time();
    m1.type = visualization_msgs::Marker::SPHERE_LIST;
    m1.action = visualization_msgs::Marker::ADD;
    m1.id = 0;
    m1.ns = "Best Particle";
    m1.scale.x = 0.35;
    m1.scale.y = 0.35;
    m1.scale.z = 0.35;
    m1.color.a = 1.0;
    m1.color.r = 0.0;
    m1.color.g = 0.0;
    m1.color.b = 1.0;
    
    float max_weight = _particles[0].getWeight();
    int id = 0;
	for (unsigned int ii = 0; ii < _particles_number; ii ++ ) 
	{
		if (_particles[ii].getWeight() > max_weight)
		{
			max_weight = _particles[ii].getWeight();
			id = ii;
		}
	}
	
	geometry_msgs::Point p1;
	p1.x = _particles[id].getX();
	p1.y = _particles[id].getY();
	m1.points.push_back(p1);
	
	ROS_INFO_STREAM("Best particle: x = " << _particles[id].getX() <<
		" y = " << _particles[id].getY() << " theta = " << _particles[id].getTheta());
	 _visualization_pub.publish(m1);
	 
	 
	 
	 _best_particle_x = _particles[id].getX();
	 _best_particle_y = _particles[id].getY();
	 _best_particle_theta = _particles[id].getTheta();
}

float ParticleFilter::getBestParticleX()
{
	return _best_particle_x;
}

float ParticleFilter::getBestParticleY()
{
	return _best_particle_y;
}

float ParticleFilter::getBestParticleTheta()
{
	return _best_particle_theta;
}



