/*
 *  Agent.cpp
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */

#include "../include/PowerLawAgent.h"
#include "../include/SimulationEngine.h"

namespace TTC {

	Agent::Agent()
	{
		_enabled = false;
		_proximityToken = NULL;
	}

	Agent::~Agent()
	{
	}

	void Agent::destroy()
	{
		if (_proximityToken!= NULL)
		{
			delete _proximityToken;
			_proximityToken = 0x0;
		}
	}


	void Agent::init(const AgentInitialParameters& initialConditions)
	{
		// initialize the agent based on the initial conditions
		_id = simEngine->getNumAgents();
		_position = initialConditions.position;
		_velocity = initialConditions.velocity;
		_goal = initialConditions.goal;
		_radius = initialConditions.radius;
		_prefSpeed = initialConditions.prefSpeed;
		_maxAccel = initialConditions.maxAccel;
		_neighborDist = initialConditions.neighborDist;
		_k = initialConditions.k;
		_ksi = initialConditions.ksi;
		_m = initialConditions.m;
		_t0 = initialConditions.t0;
		_goalRadiusSq = initialConditions.goalRadius*initialConditions.goalRadius;
		
		_enabled = true;
		//add to the database
		_proximityToken = simEngine->getSpatialDatabase()->allocateToken(this);
		// notify proximity database that our position has changed
		_proximityToken->updateForNewPosition (_position);
		
	}


	void Agent::doStep()
	{
// 		_vPref = _goal - _position;
// 		float distSqToGoal = _vPref.lengthSqr();
// 
// 		if (distSqToGoal < _goalRadiusSq)
// 		{
// 				destroy();
// 				_enabled = false;
// 				return;
// 		}
// 
// 		 // compute preferred velocity
// 		_vPref *= _prefSpeed/sqrtf(distSqToGoal);

		// compute the new velocity of the agent
		computeForces();
	}

	void Agent::computeForces()
	{

		//driving force
		_F = (_vPref - _velocity)/_ksi;

		// Compute new neighbors of agent;
		_proximityNeighbors.clear();
		_proximityToken->findNeighbors(_position, _velocity, _neighborDist, _proximityNeighbors, false);
		
		// compute the anticipatory force from each neighbor
		for (unsigned int i=0; i < _proximityNeighbors.size(); ++i)
		{
						
			const Agent* other = static_cast<Agent*>(_proximityNeighbors[i]);
			const float distanceSq = (other->position() - _position).lengthSqr();
			float radiusSq = Sqr(other->radius() + _radius);
			if (this != other && distanceSq != radiusSq)
			{	
				// if agents are actually colliding use their separation distance 
				if (distanceSq < radiusSq)
					radiusSq  = Sqr(other->radius() + _radius - sqrtf(distanceSq));
					
				const Vector2D w = other->position() - _position;
				const Vector2D v = _velocity - other->velocity();
				const float a = v*v;
				const float b = w*v;	
				const float c = w*w - radiusSq;
				float discr = b*b - a*c;
				if (discr > .0f && (a<-_EPSILON || a>_EPSILON))
				{
					discr = sqrtf(discr);
					const float t = (b - discr) / a;
					if (t>0){
	    				_F += -_k*exp(-t/_t0)*(v - (b*v - a*w)/discr)/(a*powf(t,_m))*(_m/t + 1/_t0);		
					}
				}
			}
			
		}

		//anticipatory forces from static obstacles
		for (unsigned int i=0; i < simEngine->getObstacles().size(); ++i)
		{
			const LineObstacle* obstacle = simEngine->getObstacle(i);
			Vector2D n_w = closestPointLineSegment(obstacle->p1(), obstacle->p2(), _position) - _position;
			float d_w = n_w.lengthSqr();

			if (_velocity * n_w <0 || d_w == Sqr(_radius) || d_w > Sqr(_neighborDist)) // Agent is moving away from obstacle, already colliding or obstacle too far away
				continue;
			
			const float radius = d_w < Sqr(_radius)? sqrtf(d_w):_radius; // correct the radius, if the agent is already colliding	

			const float a= _velocity*_velocity;
			bool discCollision = false, segmentCollision = false;
			float t_min = _INFTY;
	        
			float c, b, discr; 
			float b_temp, discr_temp, c_temp, D_temp;
			Vector2D w_temp, w, o1_temp, o2_temp, o_temp, o, w_o;


			// time-to-collision with disc_1 of the capped rectangle (capsule)
			w_temp = obstacle->p1() - _position;
			b_temp = w_temp*_velocity;	
			c_temp = w_temp*w_temp - (radius*radius);
			discr_temp = b_temp*b_temp - a*c_temp;
			if (discr_temp > .0f && (a<-_EPSILON || a>_EPSILON))
			{
				discr_temp = sqrtf(discr_temp);
				const float t = (b_temp - discr_temp) / a;
				if (t>0){
					t_min = t;
					b = b_temp;
					discr = discr_temp;
					w = w_temp;
					c = c_temp;
					discCollision = true;
				}
			}
					
			// time-to-collision with disc_2 of the capsule
			w_temp = obstacle->p2() - _position;
			b_temp = w_temp*_velocity;	
			c_temp = w_temp*w_temp - (radius*radius);
			discr_temp = b_temp*b_temp - a*c_temp;
			if (discr_temp > .0f && (a<-_EPSILON || a>_EPSILON))
			{
				discr_temp = sqrtf(discr_temp);
				const float t = (b_temp - discr_temp) / a;
				if (t>0 && t < t_min){
					t_min = t;
					b = b_temp;
					discr = discr_temp;
					w = w_temp;
					c = c_temp;
					discCollision = true;
				}
			}

			// time-to-collision with segment_1 of the capsule
			o1_temp = obstacle->p1() + radius * obstacle->normal();
			o2_temp = obstacle->p2() + radius * obstacle->normal();
			o_temp = o2_temp - o1_temp;

			 D_temp = det(_velocity, o_temp);
			 if (D_temp != 0)   
			 {
				float inverseDet = 1.0f/D_temp;	
				float t = det(o_temp, _position - o1_temp) * inverseDet;
				float s = det(_velocity,_position - o1_temp) * inverseDet;
				if (t>0 && s>=0 && s <=1 && t<t_min)
				{
					t_min = t;
					o = o_temp;
					w_o = _position - o1_temp;
					discCollision = false;
					segmentCollision = true;
				}
			 }

			// time-to-collision with segment_2 of the capsule
			o1_temp = obstacle->p1() - radius * obstacle->normal();
			o2_temp = obstacle->p2() - radius * obstacle->normal();
			o_temp = o2_temp - o1_temp;

			 D_temp = det(_velocity, o_temp);
			 if (D_temp != 0)   
			 {
				float inverseDet = 1.0f/D_temp;	
				float t = det(o_temp, _position - o1_temp) * inverseDet;
				float s = det(_velocity,_position - o1_temp) * inverseDet;
				if (t>0 && s>=0 && s <=1 && t<t_min)
				{
					t_min = t;
					o = o_temp;
					w_o = _position - o1_temp;
					discCollision = false;
					segmentCollision = true;
				}
			 }

			 if (discCollision)
			 {
				_F += -_k*exp(-t_min/_t0)*(_velocity - (b*_velocity - a*w)/discr)/(a*powf(t_min,_m))*(_m/t_min + 1/_t0);		
			 }
			 else if (segmentCollision)
			 {
				_F += _k*exp(-t_min/_t0)/(powf(t_min,_m)*det(_velocity,o))*(_m/t_min + 1/_t0)*Vector2D(-o.y, o.x);
			 }

		}	
			
		
	}


	void Agent::update()
	{
		// update the agents
		Vector2D acceleration = _F;
		clamp(acceleration, _maxAccel);
		_velocity = _velocity + acceleration * simEngine->getTimeStep();
		_position += _velocity * simEngine->getTimeStep();
		
		
		// notify proximity database that our position has changed
		_proximityToken->updateForNewPosition (_position);
		
	}
}	 
