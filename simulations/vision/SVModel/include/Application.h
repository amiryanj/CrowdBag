/**
 * @file Application.h
 * @author Teofilo Dutra
 */

#ifndef _APPLICATION_H_
#define _APPLICATION_H_

#include "CrowdModel.h"
#include "Agent.h"

class Application
{
public:
    
    /** Default constructor. */
    Application() {};
    
    /** Destructor. */
    ~Application();
    
    /** Crowd Model. */
    CrowdModel *m_crowdModel;
    
    /** Vector of agents. */
    vector<Agent*> m_agents;
    
    /**
     * Prepares the world. In this method the user must adds a scenario, the initial models, lights, grids, this kind of stuff.
     */
    void prepareWorld();
    
    /**
     * Does a simulator step. In this method the user must call the simulator step.
     */
    void doSimulatorStep();
    
    /**
     * Adds an agent with default properties.
     *
     * @param position Agent position.
     * @param goalPosition Agent's goal position.
     */
    void addDefaultAgent(vec3 position, vec3 goalPosition);
};

#endif
