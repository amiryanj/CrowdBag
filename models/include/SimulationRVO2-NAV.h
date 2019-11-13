#ifndef CRAAL_SIMULATIONRVO2NAV_H_
#define CRAAL_SIMULATIONRVO2NAV_H_

#ifdef UBER

#include <cstdlib>

#include "rvo2/RVO.h"
#include "Simulation.h"
#include "ubermodel/Navigation.h"

namespace craal {

/**
 * RVO2 model implementation of @ref Simulation.
 */
class SimulationRVO2NAV : public Simulation
{
private:
	RVO::RVOSimulator * g_sim;
	float *g_gx, *g_gy;
	
	bool g_useEnvironment;
	std::string g_envFile;
	bool g_reloadGoals;
	
	uberModel::Navigation * g_nav;
	uberModel::Path ** g_paths;
	
	void p_delete2();

public:
	SimulationRVO2NAV();
	virtual ~SimulationRVO2NAV();
	
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

#endif /* CRAAL_SIMULATIONRVO2NAV_H_ */
#endif
