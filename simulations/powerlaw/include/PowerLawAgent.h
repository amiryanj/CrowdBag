/*
 *  Agent.h
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */
 

/*!
 *  @file       Agent.h
 *  @brief      Contains the Agent class.
 */

#pragma once
#include <map>
#include "AgentInitialParameters.h"
#include "LineObstacle.h"
#include "proximitydatabase/Proximity2D.h"
using std::vector;

namespace TTC {

	/*!
	 * @class  Agent
	 * @brief  Simulates the anticipatory behavior of an agent.
	 */
	class Agent: public ProximityDatabaseItem
	{
	public:
		Agent();
		~Agent();
		void init(const AgentInitialParameters & initialConditions);
		void update();
		void doStep();

		/// @name Agent functionality
		//@{
		bool isAgent() { return true;}
		/// Returns true if the agent is active.
		bool enabled() const {return _enabled;}

		 
		/// Returns the position of the agent.  
		Vector2D position () const {return _position;}
		/// Returns the velocity of the agent.  
		Vector2D velocity () const {return _velocity;}
		/// Returns the goals of the agent.  
		Vector2D goal() const {return _goal;}
		/// Returns the preferred velocity of the agent.  
		Vector2D vPref() const {return _vPref;}
		/// Returns the preferred speed of the agent.  
		float prefSpeed () const {return _prefSpeed;}
		/// Returns the maximum acceleration of the agent.  
		float maxAccel () const {return _maxAccel;}
		/// Returns the radius of the agent.  
		float radius() const {return _radius;}
		/// Returns the id of the agent.  
		int id () const {return _id;}
		/*!
		 *  @brief Sets the preferred velocity of the agent to a specific value.  
		 *  @param vpref The new preferred velocity of the agent
		 */
		void setPreferredVelocity(const Vector2D& vPref){_vPref = vPref;}
		//@}

	protected:
		/// Computes the neighbors which will be considered for computing the anticipatory forces
		inline void computeNeighbors();
		/*! \brief Computes the forces exerted on the agent at each simulation step
		 *  
		 *  Our simulation model includes the following three forces:
		 *  <ul>
		 *		<li>A driving force \f$\mathbf{F}_i\f$ indicating the preference of the agent \f$i\f$ to walk in a certain direction at a certain speed, as defined in [D. Helbing, I. Farkas, and T. Vicsek, Nature 407, 487 (2000)], \f$	\mathbf{F}_{i} = \frac{\mathbf{v}^{pref}_{i}-\mathbf{v}_{i}}{\xi}\f$. 
		 *      This force can be replaced by a self-propelled force to simulate agents with no preferred direction of motion following the approach of [D. Grossman, I. S. Aranson, and E. B. Jacob, New J. Phys. 10, 023036 (2008).]
		 </li>
		 *		<li>The agent-agent interaction force \f$\mathbf{F}_{ij}\f$ derived in Eq. (S2) of the Supplemental material. </li>
		 *		<li>A similar force \f$\mathbf{F}_{iO}\f$ acting on agent \f$i\f$ as a result of interaction with each static obstacle <i>O</i> present in the environment. 
		 *		In our simulations, we generally assume that obstacles are modeled as a collection of line segments. Then, \f$\mathbf{F}_{iO} = -\nabla_{\mathbf{r}}\left( k\tau^{-2}e^{-\tau/\tau_0} \right)\f$, 
		 *      but now \f$\tau\f$ denotes the minimal intersection time between the ray \f$\mathbf{x}_{i} + t\mathbf{v}_{i}, t >0 \f$ and the 2D capsule resulting after sweeping \f$O\f$ with the disc of the agent.      
		 *		</li>
		 *	</ul>
		 */
		inline void computeForces();
		/// Destroy the agent.
		inline void destroy();
		

	public:
		/// the preferred velocity of the character
		Vector2D _vPref;
		/// Determine whether the charater is enabled;
		bool _enabled;
		/// The position of the character. 
		Vector2D _position;
		/// The goal of the character. 
		Vector2D _goal;
		/// The velocity of the character
		Vector2D _velocity;
		/// The radius of the character.
		float _radius;
		/// The id of the character. 
		int _id;
		/// The preferred speed of the character. 
		float _prefSpeed;
		/// The maximum acceleration of the character.
		float _maxAccel;
		/// The goal radius of the character
		float _goalRadiusSq;
		/// a pointer to this interface object for the proximity database
		ProximityToken* _proximityToken;
		/// The proximity neighbors
		std::vector<ProximityDatabaseItem*> _proximityNeighbors;
		
		//additional parameters for the approach
		/// The maximum distance from the agent at which an object will be considered.
		float  _neighborDist;
		/// The final force acting on the agent
		Vector2D _F;
		/// The scaling constant k of the anticipatory law
		float _k;
		/// The exponential cutoff term tau_0
		float _t0;
		/// The exponent of the power law (m = 2 in our analysis)
		float _m;
		/// Relaxation time for the driving force
		float _ksi;
		
	  	
	};
}