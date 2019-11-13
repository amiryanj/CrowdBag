#ifndef CRAAL_SIMULATIONRVO23D_H_
#define CRAAL_SIMULATIONRVO23D_H_

#include "../../simulations/rvo2-3d/include/RVO3D.h"
#include "../../craal-core/include/Simulation3D.h"

namespace craal {

/**
 * RVO2 3D model implementation of @ref Simulation3D.
 */
class SimulationRVO23D : public Simulation3D
{
private:
    RVO3D::RVOSimulator * g_sim;
    float *g_gx, *g_gy, *g_gz;

    void p_delete2();

public:
    SimulationRVO23D();
    virtual ~SimulationRVO23D();

    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y, float s_z);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y, float s_z);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y, float s_z);

    virtual void doStep(float s_dt = 0.01);
};

}

#endif /* CRAAL_SIMULATIONRVO2_H_ */
