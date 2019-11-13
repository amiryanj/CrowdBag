#ifndef CRAAL_SIMULATIONHYPER_H_
#define CRAAL_SIMULATIONHYPER_H_
#ifdef ORACLE

#include "Oracle.h"
#include "Simulation.h"

namespace craal {

/**
 * HyperModel implementation of @ref Simulation.
 */
class SimulationHyper : public Simulation
{
private:
	hyperModel::Oracle * g_oracle;
	bool g_first;

public:
	SimulationHyper();
	virtual ~SimulationHyper();
	
	virtual void init();
	virtual void reset();
	
	virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);
	
	virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
	virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
	virtual void setGoal(int s_indPedestrian, float s_x, float s_y);
	
	virtual void doStep(float s_dt = 0.01);
};

}

#endif /* CRAAL_SIMULATIONHYPER_H_ */

#endif
