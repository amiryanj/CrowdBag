/**
 * @file Application.cpp
 * @author Teofilo Dutra
 */

#include "Application.h"

Application::~Application()
{
    SAFE_DELETE(m_crowdModel)
    
    for (size_t agentIndex = 0; agentIndex < m_agents.size(); agentIndex++)
        SAFE_DELETE(m_agents[agentIndex])
        
    m_agents.clear();
}

void Application::addDefaultAgent(vec3 position, vec3 goalPosition)
{
    Agent *agent = new Agent();
    
    agent->setRadius(0.5 * Utils::getMeter());
    agent->setHeight(1.7 * Utils::getMeter());
    agent->setComfortSpeed(1.5   * Utils::getMeter());
    agent->setOrientation(glm::normalize(goalPosition-position));
    
    vec3 velocity = agent->getComfortSpeed() * agent->getOrientation();
    
    agent->setVelocity(velocity);
    agent->setPosition(position);
    agent->setGoalPosition(goalPosition);
    
    m_agents.push_back(agent);
}

void Application::prepareWorld()
{
    Utils::setMeter(1.0f);
    
    m_crowdModel = new CrowdModel();
    
    // Three scenarios for tests. 0 = circle, 1 = opposite directions, 2 = crossing.
    
    int scenario = 0;
    
    switch (scenario) {
        case 0:
        {
            size_t agentsCount = 2;
            mFloat radius = 20.0f;
            mFloat angle = M_PI/(agentsCount/2.0);
            vec3 agentPosition(radius, 0, 0);
            vec3 goalPosition = -agentPosition;
            
            for (size_t agentIndex = 0; agentIndex < agentsCount; agentIndex++)
            {
                addDefaultAgent(agentPosition, goalPosition);
                
                float x = agentPosition.x * cos(angle) + agentPosition.z * sin(angle);
                float z = agentPosition.x * (-sin(angle)) + agentPosition.z * cos(angle);
                
                agentPosition = vec3(x, 0.0, z);
                goalPosition = - agentPosition;
            }
        }
            break;
        case 1:
        {
            size_t rows = 5, columns = 5;
            mFloat space = 2.5;
            vec3 initial(-20.0, 0.0, -6.0);
            
            for (size_t rowIndex = 0 ; rowIndex < rows; rowIndex++)
            {
                for (size_t columnIndex = 0; columnIndex < columns; columnIndex++)
                {
                    vec3 agentPosition(initial.x + columnIndex * space, 0.0, initial.z + rowIndex * space);
                    vec3 goalPosition = agentPosition;
                    goalPosition.x = -goalPosition.x;
                    
                    addDefaultAgent(agentPosition, goalPosition);
                    addDefaultAgent(goalPosition,  agentPosition);
                }
            }
        }
            break;
        case 2:
        {
            size_t rows = 5, columns = 5;
            mFloat space = 2.5;
            vec3 initial1(-20.0, 0.0, -6.0);
            vec3 initial2(-6.0, 0.0, 20.0);
            
            for (size_t rowIndex = 0 ; rowIndex < rows; rowIndex++)
            {
                for (size_t columnIndex = 0; columnIndex < columns; columnIndex++)
                {
                    vec3 agentPosition(initial1.x + columnIndex * space, 0.0, initial1.z + rowIndex * space);
                    vec3 goalPosition = agentPosition;
                    goalPosition.x = -goalPosition.x;
                    addDefaultAgent(agentPosition, goalPosition);
                    
                    agentPosition = vec3(initial2.x + rowIndex * space, 0.0, initial2.z - columnIndex * space);
                    goalPosition = agentPosition;
                    goalPosition.z = -goalPosition.z;
                    addDefaultAgent(goalPosition,  agentPosition);
                }
            }
        }
            break;
    }
    
    m_crowdModel->setTimeStep(0.05f);
    m_crowdModel->setAgents(m_agents);
}

void Application::doSimulatorStep()
{
    m_crowdModel->doStep();
}
