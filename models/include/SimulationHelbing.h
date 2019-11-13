#ifndef CRAAL_SIMULATIONHELBING_H_
#define CRAAL_SIMULATIONHELBING_H_

#include "../../simulations/helbing/include/Helbing.h"
#include "../../craal-core/include/Simulation.h"

namespace craal {

/**
 * Social-forces model implementation of @ref Simulation.
 */
class SimulationHelbing : public Simulation
{
private:
    helbing::HelbingSim * g_sim;
    float *g_gx, *g_gy;

    void p_delete2();

public:
    SimulationHelbing();
    virtual ~SimulationHelbing();

    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y);

    virtual void doStep(float s_dt = 0.01);

    void setAgentRadius(int s_indPedestrian, float value);
    void setAgentSpeed(int s_indPedestrian, float value);

};

}

#endif /* CRAAL_SIMULATIONHELBING_H_ */
