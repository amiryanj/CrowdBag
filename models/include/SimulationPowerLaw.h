#ifndef CRAAL_SIMULATIONPOWERLAW_H_
#define CRAAL_SIMULATIONPOWERLAW_H_

#include "../../simulations/powerlaw/include/SimulationEngine.h"
#include "../../craal-core/include/Simulation.h"

namespace craal {

/**
 * Power law implementation of @ref Simulation.
 */
class SimulationPowerLaw : public Simulation
{
private:
    TTC::SimulationEngine * g_sim;
    float *g_gx, *g_gy;

    void p_delete2();

public:
    SimulationPowerLaw();
    virtual ~SimulationPowerLaw();

    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y);

    virtual void doStep(float s_dt = 0.01);

    void setAgentPrefSpeed(int s_indPedestrian, float value);
    void setAgentNeighborDist(int s_indPedestrian, float value);
    void setAgentRadius(int s_indPedestrian, float value);
    void setAgentT0(int s_indPedestrian, float value);
    void setAgentK(int s_indPedestrian, float value);
    void setAgentKsi(int s_indPedestrian, float value);
    void setAgentM(int s_indPedestrian, float value);
    void setAgentMaxAcceleration(int s_indPedestrian, float value);


};

}

#endif /* CRAAL_SIMULATIONPOWERLAW_H_ */
