#ifndef CRAAL_SIMULATIONUBER_H_
#define CRAAL_SIMULATIONUBER_H_

#ifdef UBER_MODEL
#include "Simulation.h"

#include "ubermodel/core/Oracle.h"
#include "ubermodel/implementation/DistortionRadius.h"
#include "ubermodel/implementation/DistortionReferentialA2W.h"
#include "ubermodel/implementation/DistortionReferentialW2A.h"
#include "ubermodel/implementation/DistortionFFReferentialA2W.h"
#include "ubermodel/implementation/DistortionFFReferentialW2A.h"
#include "ubermodel/implementation/DistortionSpeed.h"
#include "ubermodel/implementation/DistortionSpeedUncertainty.h"
#include "ubermodel/implementation/DistortionTHorizon.h"
#include "ubermodel/implementation/DistortionTimeUncertainty.h"
#include "ubermodel/implementation/DistributionGaussian.h"
#include "ubermodel/implementation/DistributionSolid.h"
#include "ubermodel/utility/Navigation.h"
#include "ubermodel/utility/Navigator.h"

namespace craal {

/**
 * UberModel implementation of @ref Simulation.
 */
class SimulationUber : public Simulation
{
private:
    uberModel::Oracle * g_sim;
    float *g_gx, *g_gy;

    bool g_useEnvironment;
    std::string g_envFile;
    bool g_reloadGoals;

    std::vector<float> g_obstacles;

    uberModel::Navigator * g_navigator;

    void p_delete2();

public:
    SimulationUber();
    virtual ~SimulationUber();

    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y);

    virtual void doStep(float s_dt = 0.01);

    virtual void updateSpecific();
};

}

#endif

#endif /* CRAAL_SIMULATIONUBER_H_ */
