#ifndef CRAAL_SIMULATIONRVO2_H_
#define CRAAL_SIMULATIONRVO2_H_

#include "../../simulations/rvo2/include/RVO.h"
#include "../../craal-core/include/Simulation.h"

namespace craal {

/**
 * RVO2 model implementation of @ref Simulation.
 */
class SimulationRVO2 : public Simulation
{
private:
    RVO::RVOSimulator * g_sim;
    float *g_gx, *g_gy;

    void p_delete2();

public:
    SimulationRVO2();
    virtual ~SimulationRVO2();

    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y);

    virtual void doStep(float s_dt = 0.01);

    void setAgentMaxSpeed(int s_indPedestrian, float value);
    void setAgentNeighborDist(int s_indPedestrian, float value);
    void setAgentRadius(int s_indPedestrian, float value);
    void setAgentTimeHorizon(int s_indPedestrian, float value);
    void setAgentTimeHorizonObst(int s_indPedestrian, float value);

};

}

#endif /* CRAAL_SIMULATIONRVO2_H_ */
