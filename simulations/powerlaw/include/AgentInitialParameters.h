/*
 *  AgentInitialParameters.h
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */

/*!
 *  @file       AgentInitialParameters.h
 *  @brief      Contains the AgentInitialParameters struct.
 */

#pragma once
#include "util/Vector2D.h"

namespace TTC {

	/** 
	 * @brief The initial parameters for a single agent.
	*/
	struct AgentInitialParameters {
		/// The start position of the agent. 
		Vector2D position;
		/// The goal position of the agent. 
		Vector2D goal;
		/// The initial velocity of the agent.
		Vector2D velocity;		
		/// The radius of the agent.
		float radius;
		/// The preferred speed of the agent. 
		float prefSpeed;
		/// The maximum acceleration of the agent.
		float maxAccel;
		/// How close to the goal the agent should be to stop the simulation. 
		float goalRadius;
		/// The nn distance used to speeed up proximity queries. 
		float neighborDist;
		/// The scaling constant 
		float k;
		/// The relaxation time
		float ksi;
		/// The exponent of the power law
		float m;
		/// The exponential cutoff point
		float t0;				
	};

}