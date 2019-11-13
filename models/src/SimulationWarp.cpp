#include "../include/SimulationWarp.h"

#ifdef WARP_MODEL
namespace craal {

SimulationWarp::SimulationWarp() :
	g_sim(0)
{
	addParam("AgentCorrectionMagnitude", 2.0, PDF(0.0, 5.0, PDF::NORMAL, 2.5, 2.5));
	addParam("Cautiousness", 0.8, PDF(0.0, 2.0, PDF::NORMAL, 1.0, 1.0));
	addParam("GoalCorrectionMagnitude", 0.5, PDF(0.0, 2.0, PDF::NORMAL, 1.0, 1.0));
	addParam("GoalToAgentImportance", 0.09, PDF(0.0, 0.5, PDF::NORMAL, 0.2, 0.3));
	addParam("MaximumSpeed", 2.0, PDF(1.0, 3.0, PDF::NORMAL, 2.0, 1.0));
	addParam("PreferredSpeed", 1.4, PDF(1.0, 2.0, PDF::NORMAL, 1.5, 0.5));
	addParam("Radius", 0.25, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
	addParam("TimeHorizon", 3.0, PDF(2.0, 5.0, PDF::NORMAL, 3, 1));
	addParam("TimeUncertainty", 0.5, PDF(0.0, 1.0, PDF::NORMAL, 0.5, 0.5));
	
	name = "WarpDriver";
	
	agent_value_dependency("AgentCorrectionMagnitude", &g_offsets[0]);
	agent_value_dependency("Cautiousness", &g_offsets[1]);
	agent_value_dependency("GoalCorrectionMagnitude", &g_offsets[2]);
	agent_value_dependency("GoalToAgentImportance", &g_offsets[3]);
	agent_value_dependency("MaximumSpeed", &g_offsets[4]);
	agent_value_dependency("PreferredSpeed", &g_offsets[5]);
	agent_value_dependency("Radius", &g_offsets[6]);
	agent_value_dependency("TimeHorizon", &g_offsets[7]);
	agent_value_dependency("TimeUncertainty", &g_offsets[8]);
	
	agent_value_dependency("Goal", &g_goal_i);
	agent_value_dependency("CurrentPosition", &g_position_i);
	agent_value_dependency("CurrentVelocity", &g_velocity_i);
}

SimulationWarp::~SimulationWarp()
{
	p_delete2();
}

void SimulationWarp::p_delete2()
{
	if (g_sim) delete g_sim;
	g_sim = 0;
}

void SimulationWarp::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
// 	g_obstacles.push_back(s_startx);
// 	g_obstacles.push_back(s_starty);
// 	g_obstacles.push_back(s_endx);
// 	g_obstacles.push_back(s_endy);
// 	
// 	if (g_obstacles.size()/4 > g_sim->obstacles.g_nbWalls) {
// 		float minx = g_obstacles[0];
// 		float miny = g_obstacles[1];
// 		float maxx = g_obstacles[0];
// 		float maxy = g_obstacles[1];
// 		
// 		for (int i = 0; i < g_obstacles.size()/2; i++) {
// 			if (minx > g_obstacles[i*2 + 0]) minx = g_obstacles[i*2 + 0];
// 			if (miny > g_obstacles[i*2 + 1]) miny = g_obstacles[i*2 + 1];
// 			if (maxx < g_obstacles[i*2 + 0]) maxx = g_obstacles[i*2 + 0];
// 			if (maxy < g_obstacles[i*2 + 1]) maxy = g_obstacles[i*2 + 1];
// 		}
// 		
// 		g_sim->obstacles.init(
// 			g_obstacles.size()/4,
// 			minx,
// 			maxx,
// 			miny,
// 			maxy,
// 			0.25,
// 			0.06283185307179587
// 		);
// 		
// 		memcpy(
// 			g_sim->obstacles.g_walls,
// 			g_obstacles.data(),
// 			g_obstacles.size()*sizeof(float)
// 		);
// 		
// 		g_sim->obstacles.compute();
// 	}
}

void SimulationWarp::init()
{
	p_delete2();
	
	warp::AlgorithmFactory f;
	
	f.add_value(new warp::values::AgentCorrectionMagnitude());
	f.add_value(new warp::values::Blocking());
	f.add_value(new warp::values::Cautiousness());
	f.add_value(new warp::values::CoarseSize());
	f.add_value(new warp::values::ComputingDisplayProbabilities());
	f.add_value(new warp::values::CurrentPosition());
	f.add_value(new warp::values::CurrentVelocity());
	f.add_value(new warp::values::DefaultNormalizedCurrentVelocity());
	f.add_value(new warp::values::DisplayProbabilities());
// 	f.add_value(new warp::values::FlowAgent());
// 	f.add_value(new warp::values::FlowGoal());                  // mutually exclusive with Goal, RandomGoal
// 	f.add_value(new warp::values::FlowManager());
	f.add_value(new warp::values::Goal());                      // mutually exclusive with FlowGoal, RandomGoal
	f.add_value(new warp::values::GoalCorrectionMagnitude());
	f.add_value(new warp::values::GoalSize());
	f.add_value(new warp::values::GoalToAgentImportance());
	f.add_value(new warp::values::MaximumSpeed());
	f.add_value(new warp::values::Navigate());
	f.add_value(new warp::values::NextVelocity());
	f.add_value(new warp::values::NormalizedCurrentVelocity());
	f.add_value(new warp::values::PreferredSpeed());
	f.add_value(new warp::values::PreferredVelocity());
	f.add_value(new warp::values::PreferredVelocityCentered());
	f.add_value(new warp::values::Priority());
	f.add_value(new warp::values::Radius());
// 	f.add_value(new warp::values::RandomGoal());                // mutually exclusive with Goal, FlowGoal
// 	f.add_value(new warp::values::RandomGoalZone());
	f.add_value(new warp::values::Recorder());
	f.add_value(new warp::values::SamplingStep());
	f.add_value(new warp::values::Speed());
	f.add_value(new warp::values::TimeHorizon());
	f.add_value(new warp::values::TimeUncertainty());
	f.add_value(new warp::values::UpdateFlowEntry());
	f.add_value(new warp::values::WallWidth());
	
	f.add_operator<warp::operators::GaussianField>();
	f.add_operator<warp::operators::TimeUncertainty>();
	f.add_operator<warp::operators::TimeHorizon>();
	f.add_operator<warp::operators::Radius>();
// 	f.add_operator<warp::operators::ShapeWall>();
	f.add_operator<warp::operators::Speed>();
	f.add_operator<warp::operators::GlobalSpaceToPerceived>();
	f.add_operator<warp::operators::GlobalSpace>();
	f.add_operator<warp::operators::PerceivingToGlobalSpace>();
// 	f.add_operator<warp::operators::OperatorDisablerForProbabilities>();
	
	f.set_driver<warp::WDCulledGradientDescent>();
	
	f.add_access(this);
	
	g_sim = f.create(g_nPedestrian);
	
	std::vector<float> grid_precision;
	std::vector<float> grid_threshold;
	std::vector<int> grid_nsampled;
	
	grid_precision.push_back(1000.0);
	
	grid_threshold.push_back(1000.0);
	
	grid_nsampled.push_back(std::min(100, g_sim->data->get_nagents()));
	
	((warp::WDCulledGradientDescent*)g_sim)->init(
		warp::Point(-500.0, -500.0, 0.0),
		warp::Point(500.0, 500.0, 0.0),
		grid_precision,
		grid_threshold,
		grid_nsampled
	);
	
	for (int i = 0; i < g_nPedestrian; i++) {
		setGoal(i, 0.0, 0.0);
	}
}

void SimulationWarp::reset()
{
	for (int i = 0; i < g_nPedestrian; i++) {
		data->set_active_agent(i);
		
		for (int j = 0; j < 9; j++) {
			data->active_agent_value<float>(g_offsets[j]) = g_params[i*9 + j];
		}
	}
}

void SimulationWarp::setPosition(int s_indPedestrian, float s_x, float s_y)
{
	warp::Point & p = data->agent_value<warp::Point>(s_indPedestrian, g_position_i);
	
	p.x = s_x;
	p.y = s_y;
}

void SimulationWarp::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
	warp::Point & v = data->agent_value<warp::Point>(s_indPedestrian, g_velocity_i);
	
	v.x = s_x;
	v.y = s_y;
}

void SimulationWarp::setGoal(int s_indPedestrian, float s_x, float s_y)
{
	warp::Point & g = data->agent_value<warp::Point>(s_indPedestrian, g_goal_i);
	
	g.x = s_x;
	g.y = s_y;
}

void SimulationWarp::doStep(float s_dt)
{
	data->compute();
	g_sim->compute_step(s_dt);
	g_sim->apply_step(s_dt);
	
	for (int i = 0; i < g_nPedestrian; i++) {
		warp::Point & p = data->agent_value<warp::Point>(i, g_position_i);
		warp::Point & v = data->agent_value<warp::Point>(i, g_velocity_i);
	
		setNextState(i, p.x, p.y, v.x, v.y);
	}
}

void SimulationWarp::updateSpecific()
{}

}

#endif // WARP_MODEL
