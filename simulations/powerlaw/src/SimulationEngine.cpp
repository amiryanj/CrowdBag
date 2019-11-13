/*
 *  SimulationEngine.cpp
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */


#include "../include/SimulationEngine.h"

#if _OPENMP
#include <omp.h>
#endif


TTC::SimulationEngine* simEngine;

namespace TTC {

	SimulationEngine::SimulationEngine()
	{
		_spatialDatabase = NULL;
		simEngine = this;

	}

	SimulationEngine::~SimulationEngine()
	{

		for(vector<Agent*>::iterator it = _agents.begin(); it != _agents.end(); ++it)
		{
			delete *it;
			*it = 0x0;

		}
		
		for(vector<LineObstacle*>::iterator it = _obstacles.begin(); it != _obstacles.end(); ++it)
		{
			delete *it;
			*it = 0x0;
		}
		
			
		if (_spatialDatabase!=NULL)
		{
			delete _spatialDatabase;
			_spatialDatabase = 0x0;
		}

	}

	void SimulationEngine::init(float xRange, float yRange)
	{
		_iteration = 0;
		_globalTime = 0;
		_reachedGoals = false;
		
		//initialize the database
		_spatialDatabase = new SpatialProximityDatabase(Vector2D(), Vector2D(xRange,yRange), Vector2D(10.0f, 10.0f));
		
	}


	void SimulationEngine::updateSimulation()
	{
		_reachedGoals = true;

		#pragma omp parallel for
		for (int i = 0; i < (int)_agents.size(); ++i)
		{
			if (_agents[i]->enabled())
			{
				_agents[i]->doStep();
				_reachedGoals = false;
			}
		}


		#pragma omp parallel for
		for (int i = 0; i < (int)_agents.size(); ++i)
		{
			if (_agents[i]->enabled())
			{
				_agents[i]->update();
			}
		}

		_globalTime += _dt;
		_iteration++;
	}


	bool SimulationEngine::endSimulation()
	{
		return _reachedGoals || _iteration >= _maxSteps;
	}

	void SimulationEngine::updateVisualisation()
	{
	  // Output the current global time. 
		std::cout << "Time: " << _globalTime << std::endl;

	  // Output the current position of all the agents. 
	  for (unsigned int i = 0; i < _agents.size(); ++i)
		  if (_agents[i]->enabled())
			  std::cout << i << ": " <<_agents[i]->position() << " ";
	  std::cout << std::endl;
	}
	
	
	void SimulationEngine::exportSimulation(std::ofstream& file)
	{   
		for (unsigned int i = 0; i < _agents.size(); ++i)
		{
			if (_agents[i]->enabled())
				file << _agents[i]->id() << ","  << _agents[i]->position().x << "," << _agents[i]->position().y <<"," << 
				_agents[i]->radius() << "," << _globalTime <<std::endl;

		}
	
	}

	void SimulationEngine::addAgent(AgentInitialParameters& agentConditions)
	{
		Agent* newAgent = new Agent();
		if (newAgent != NULL) {
			newAgent->init(agentConditions);
			_agents.push_back(newAgent);
		}
	}

	void SimulationEngine::addObstacle(const std::pair<Vector2D, Vector2D>& lineSegment)
	{
		LineObstacle* l = new LineObstacle(lineSegment.first, lineSegment.second);
		_obstacles.push_back(l);
	}

}
