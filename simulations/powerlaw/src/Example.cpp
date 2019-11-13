/*
 *  Example.cpp
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */
 

/*!
 *  @file       Example.cpp
 *  @brief      An example of how to use this library.
 */
#include "SimulationEngine.h"
#include "conio.h"
using namespace TTC;

SimulationEngine * _engine = 0;

void destroy ()
{
	delete _engine; 
	_engine = 0x0;
}


void setupScenario()
{


	//initialize the engine, given the dimensions of the environment
	_engine->init(50, 50);

    // Specify the default parameters for agents that are subsequently added.	
	AgentInitialParameters par;

	par.k = 1.5f;
	par.ksi = 0.54f;
	par.m = 2.0f;
	par.t0 = 3.f;
	par.neighborDist = 10.f;
	par.maxAccel = 20.f; 
	par.radius = 0.5f;
	par.prefSpeed = 1.4f;
	par.goalRadius = 0.5f;
	
 
	// Add agents, specifying their start positions, goal positions and preferred speeds. The agents are distributed along the circumference of a circle
	// and have to move to their antipodal positions.
   for (int i = 0; i < 10; ++i) 
	{
		AgentInitialParameters p = par;  
		p.position = 10.f * Vector2D(cos(i * 2.0f * _M_PI / 10.0f), sin(i * 2.0f * _M_PI / 10.0f));
		p.goal = Vector2D(-p.position.x, -p.position.y);
		p.velocity = Vector2D();
		//gaussian distributed speed
		float u;
		do{
			u = (float)rand()/(float)RAND_MAX;
		} while (u >= 1.0);
		p.prefSpeed += sqrtf( -2.f * logf( 1.f - u)) * 0.1f * cosf(2.f*_M_PI*(float)rand()/(float)RAND_MAX);
		_engine->addAgent(p);
   }

   
   /* Static obstacles can be added as follows:
	*   std::pair<Vector2D, Vector2D> lineSegment = std::make_pair(Vector2D(-20, -20), Vector2D(20, -20));
	*   _engine->addObstacle(lineSegment);
    */
}

int main(int argc, char **argv)
{	
	//default parameters
	int numFrames = 5000;
	float dt = 0.005f;
	
	//load the engine
	_engine = new SimulationEngine(); 
	_engine->setTimeStep(dt);
	_engine->setMaxSteps(numFrames);

	// setup the scenario
	setupScenario();
	
	
	// Run the scenario
	do 
	{
		_engine->updateVisualisation();
		_engine->updateSimulation();
	} while ( !_engine->endSimulation());	


	//press a key to terminate
	while (!_kbhit())
	{
		 
	}

	//destroy the environment
	destroy();

	return 0;
		
}


