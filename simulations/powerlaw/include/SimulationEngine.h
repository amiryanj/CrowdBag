/*
 *  SimulationEngine.h
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */

/*!
 *  @file       SimulationEngine.h
 *  @brief      Contains the SimulationEngine class.
 */

#pragma once

#include "./PowerLawAgent.h"
#include "./LineObstacle.h"
#include <fstream>


/*!
 * @namespace	TTC
 * @brief The namespace of the library.
 * To make sure that all functions in the library are not conflicting with any other function the library uses it's own namespace. 
 * One always needs to use this namespace to access the functions inside this library.
*/

namespace TTC {

	/*!
	 *  @class      SimulationEngine
	 *  @brief      The main class of the library.
	 */
	class SimulationEngine {
	 
	  protected:
		

		/// The time step in the simulation.
		float  _dt;
		
		/// The current time in the simulation.
		float  _globalTime;

		/// The current iteration step.
		int _iteration;
				 
		///The maximum number of simulation steps.
		int _maxSteps;

		///Determine whether all agents have reached their goals
		bool _reachedGoals;

		/// The proximity database
		SpatialProximityDatabase * _spatialDatabase;
		
		/// The agents in the simulation
		vector<Agent* >  _agents;

		/// The obstacles in the simulation
		vector<LineObstacle* >  _obstacles;
	

	  public:
		/*! 
		Default Costructor. 
		*/
		SimulationEngine();

		/*! Destructor. */
		 ~SimulationEngine();

		/*!
		 * @brief
		 * Initialization of the simulation engine. 
		 * All characters in the library have the same time step.
		 * @param xRange The range of the x-dimension of the environment.
		 * @param yRange The range of the y-dimension of the environment.
		 */
		 void init(float xRange, float yRange);


		/*! 
		 * @brief
		 *  Performs a simulation/integration step i.e. updates the acceleration, velocity and position of the simulated characters.
		 */
		 void updateSimulation();

		/*!
		 * @brief
		 * This function determines whether the simulation has to stop i.e. when all characters have reached their goals or the simulation steps have exceeded the maximum allowed number.
		 * @return The function returns true when the simulation has to stop. False otherwise. */
		 bool endSimulation();

		/*! 
		 * @brief
		 *  Outputs a simulation step in the console.
		 */
		 void updateVisualisation();
		
		/*! 
		 * @brief
		 *  Outputs a simulation step in a file.
 		 * @param file The file for exporting the simulation.
		 */
		 void exportSimulation(std::ofstream& file);
		
		/*! 
		 * @brief Add  a new agent to the simulation given its default parameters
		 * @param parameters The agent's parameters
		 */
		 void addAgent(AgentInitialParameters& parameters);

		/*! 
		 *  @brief Add  a new line obstacle to the simulation 
		 *  @param lineSegment The start and end point of the line segment
		 */
		 void addObstacle(const std::pair<Vector2D, Vector2D>& lineSegment);
		
	
		/// @name Get/Set functionality
		//@{
		 
		///	Returns a pointer to the proximity database of the engine.
		SpatialProximityDatabase* getSpatialDatabase() { return _spatialDatabase;}
	
		/// Returns the list of agents in the simulation. 
		const vector<Agent*> & getAgents()  {return _agents;}
		
		///  Returns the list of obstacles. 
		const vector<LineObstacle*> & getObstacles()  {return _obstacles;}
		
		/*! 
		 *  @brief Returns the corresponding agent given its id 
		 *  @param id The id of the agent.
		 */
		Agent* getAgent(int id) {return _agents[id];}

		/*! 
		 *  @brief Returns the corresponding line obstacle given its id. 
		 *  @param id The id of the obstacle.
		 */
		LineObstacle* getObstacle(int id) {return _obstacles[id];}
		
		/// Returns the time step of the simulation. 
		float getTimeStep() const {return _dt;}

		/*! 
			@brief Sets the time step of the simulation. 
			@param dt The time step of the simulation.
        */
		void setTimeStep( float dt) { _dt = dt; }

		  
		///  Returns the number of simulations steps. 
		int getMaxSteps() const {return _maxSteps;}

		/*! 
			@brief Sets the maximum number of simulation steps. 
			@param steps The maximum number of simulation steps.
        */
		void setMaxSteps( int steps) { _maxSteps = steps; }

		///  Returns the global time of the simulation. Initially this time is set to zero.  
		float getGlobalTime() const { return _globalTime; }

		/// Returns the number of agents in the simulation. 
		int getNumAgents() const {return _agents.size();}

		/// Returns the current simulation step.  
		int getIterationNumber() const {return _iteration;}

	   	//@}

	 };
	 
}
	 extern TTC::SimulationEngine* simEngine;


