#ifndef CRAAL_SIMULATIONWARP_H_
#define CRAAL_SIMULATIONWARP_H_

#ifdef WARP_MODEL

#include "Simulation.h"

#include "warp/core/AlgorithmFactory.h"
#include "warp/implementation/WDCulledGradientDescent.h"

#include "warp/implementation/values/AgentCorrectionMagnitude.h"
#include "warp/implementation/values/Blocking.h"
#include "warp/implementation/values/Cautiousness.h"
#include "warp/implementation/values/CoarseSize.h"
#include "warp/implementation/values/ComputingDisplayProbabilities.h"
#include "warp/implementation/values/CurrentPosition.h"
#include "warp/implementation/values/CurrentVelocity.h"
#include "warp/implementation/values/DefaultNormalizedCurrentVelocity.h"
#include "warp/implementation/values/DisplayProbabilities.h"
#include "warp/implementation/values/FlowAgent.h"
// #include "warp/implementation/values/FlowGoal.h"
// #include "warp/implementation/values/FlowManager.h"
#include "warp/implementation/values/Goal.h"
#include "warp/implementation/values/GoalCorrectionMagnitude.h"
#include "warp/implementation/values/GoalSize.h"
#include "warp/implementation/values/GoalToAgentImportance.h"
#include "warp/implementation/values/MaximumSpeed.h"
#include "warp/implementation/values/Navigate.h"
#include "warp/implementation/values/NextVelocity.h"
#include "warp/implementation/values/NormalizedCurrentVelocity.h"
#include "warp/implementation/values/PreferredSpeed.h"
#include "warp/implementation/values/PreferredVelocity.h"
#include "warp/implementation/values/PreferredVelocityCentered.h"
#include "warp/implementation/values/Priority.h"
#include "warp/implementation/values/Radius.h"
// #include "warp/implementation/values/RandomGoal.h"
// #include "warp/implementation/values/RandomGoalZone.h"
#include "warp/implementation/values/Recorder.h"
#include "warp/implementation/values/SamplingStep.h"
#include "warp/implementation/values/Speed.h"
#include "warp/implementation/values/TimeHorizon.h"
#include "warp/implementation/values/TimeUncertainty.h"
#include "warp/implementation/values/UpdateFlowEntry.h"
#include "warp/implementation/values/WallWidth.h"

#include "warp/implementation/operators/GaussianField.h"
#include "warp/implementation/operators/GlobalSpace.h"
#include "warp/implementation/operators/GlobalSpaceToPerceived.h"
#include "warp/implementation/operators/PerceivingToGlobalSpace.h"
#include "warp/implementation/operators/OperatorDisablerForProbabilities.h"
#include "warp/implementation/operators/Radius.h"
#include "warp/implementation/operators/ShapeWall.h"
#include "warp/implementation/operators/Speed.h"
#include "warp/implementation/operators/TimeHorizon.h"
#include "warp/implementation/operators/TimeUncertainty.h"

namespace craal {

/**
 * WarpDriver implementation of @ref Simulation.
 */
class SimulationWarp : public Simulation, public warp::DataAccess
{
private:
	warp::WarpDriver * g_sim;
	int g_offsets[9];
	
	int g_goal_i;
	int g_position_i;
	int g_velocity_i;
	
	std::vector<float> g_obstacles;
	
	void p_delete2();

public:
	SimulationWarp();
	virtual ~SimulationWarp();
	
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

#endif // WARP_MODEL

#endif /* CRAAL_SIMULATIONWARP_H_ */

