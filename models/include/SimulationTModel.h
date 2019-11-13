#ifndef CRAAL_SIMULATIONTMODEL_H_
#define CRAAL_SIMULATIONTMODEL_H_

#include "core/include/Simulation.h"
#include "simulations/tmodel/include/AgentSCA09.h"

namespace craal {

/**
 * Tangent model implementation of @ref Simulation.
 */
class SimulationTModel : public Simulation
{
private:
    std::vector<SCA09::Agent*> g_agents;

    void p_deleteAgents();

public:
    SimulationTModel();
    virtual ~SimulationTModel();

    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y);

    virtual void doStep(float s_dt = 0.01);
};

}

#endif /* CRAAL_SIMULATIONTMODEL_H_ */
