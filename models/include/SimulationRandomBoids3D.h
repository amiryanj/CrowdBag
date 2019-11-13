#ifndef CRAAL_SIMULATIONRANDOMBOIDS3D_H_
#define CRAAL_SIMULATIONRANDOMBOIDS3D_H_

#include "../../simulations/helbing-3d/include/Helbing3D.h"
#include "../../craal-core/include/Simulation3D.h"

#include "Sampler.h"

namespace craal {

/**
 * Boids-like 3D model implementation of @ref Simulation3D.
 */
class SimulationRandomBoids3D : public Simulation3D
{
private:
    helbing3d::HelbingSim * g_sim;
    float *g_px, *g_py, *g_pz;
    float *g_gx, *g_gy, *g_gz;
    float * g_d;

    std::string g_fileAz;
    std::string g_fileAy;
    std::string g_fileL;

    Sampler g_sampAz;
    Sampler g_sampAy;
    Sampler g_sampL;

    void p_delete2();

public:
    SimulationRandomBoids3D();
    virtual ~SimulationRandomBoids3D();

    virtual void updateSpecific();

    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y, float s_z);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y, float s_z);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y, float s_z);

    virtual void doStep(float s_dt = 0.01);
};

}

#endif /* CRAAL_SIMULATIONRANDOMBOIDS3D_H_ */
