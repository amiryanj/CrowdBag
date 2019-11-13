#include "../include/SimulationUber.h"

#ifdef UBER_MODEL
namespace craal {

SimulationUber::SimulationUber() :
	g_sim(0), g_gx(0), g_gy(0), g_useEnvironment(false),
	g_envFile("None"), g_reloadGoals(false), g_navigator(0)
{
	addParam("TemporalHorizon", 3.0, PDF(2.0, 5.0, PDF::NORMAL, 3, 1));
	addParam("PreferredSpeed", 1.4, PDF(1.0, 2.0, PDF::NORMAL, 1.5, 0.5));
	addParam("Correction", 5.0, PDF(1.0, 10.0, PDF::NORMAL, 5.0, 5.0));
	addParam("Mix", 0.05, PDF(0.001, 0.1, PDF::NORMAL, 0.05, 0.05));
	addParam("SpeedUncertaintyx", 0.25, PDF(0.0, 1.0, PDF::NORMAL, 0.5, 0.5));
	addParam("SpeedUncertaintyy", 0.25, PDF(0.0, 1.0, PDF::NORMAL, 0.5, 0.5));
	addParam("Radius", 0.25, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
	addParam("TimeUncertainty", 0.2, PDF(0.0, 0.5, PDF::NORMAL, 0.25, 0.25));
	
	addConfigurable("Use environment", new Type_bool, &g_useEnvironment);
	addConfigurable("Environment file", new Type_string, &g_envFile);
	
	name = "Uber";
}

SimulationUber::~SimulationUber()
{
	p_delete2();
}

void SimulationUber::p_delete2()
{
// 	if (g_params == 0 && g_sim) g_sim->setParams(0);
	if (g_sim) delete g_sim;
	if (g_gx) delete [] g_gx;
	if (g_gy) delete [] g_gy;
	g_sim = 0;
	g_gx = 0;
	g_gy = 0;
	if (g_navigator) delete g_navigator;
	g_navigator = 0;
}

void SimulationUber::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
	g_obstacles.push_back(s_startx);
	g_obstacles.push_back(s_starty);
	g_obstacles.push_back(s_endx);
	g_obstacles.push_back(s_endy);
	
	if (g_obstacles.size()/4 > g_sim->obstacles.g_nbWalls) {
		float minx = g_obstacles[0];
		float miny = g_obstacles[1];
		float maxx = g_obstacles[0];
		float maxy = g_obstacles[1];
		
		for (int i = 0; i < g_obstacles.size()/2; i++) {
			if (minx > g_obstacles[i*2 + 0]) minx = g_obstacles[i*2 + 0];
			if (miny > g_obstacles[i*2 + 1]) miny = g_obstacles[i*2 + 1];
			if (maxx < g_obstacles[i*2 + 0]) maxx = g_obstacles[i*2 + 0];
			if (maxy < g_obstacles[i*2 + 1]) maxy = g_obstacles[i*2 + 1];
		}
		
		g_sim->obstacles.init(
			g_obstacles.size()/4,
			minx,
			maxx,
			miny,
			maxy,
			0.25,
			0.06283185307179587
		);
		
		memcpy(
			g_sim->obstacles.g_walls,
			g_obstacles.data(),
			g_obstacles.size()*sizeof(float)
		);
		
		g_sim->obstacles.compute();
	}
}

void SimulationUber::init()
{
	p_delete2();
	
	g_sim = new uberModel::Oracle();
	g_gx = new float[g_nPedestrian];
	g_gy = new float[g_nPedestrian];
	
	for (int i = 0; i < g_nPedestrian; i++) {
		g_gx[i] = 0;
		g_gy[i] = 0;
	}
	
	g_sim->setNAgents(g_nPedestrian);
// 	g_sim->addDistortion(new uberModel::DistortionReferentialA2W());
// 	g_sim->addDistortion(new uberModel::DistortionReferentialW2A());
	g_sim->addDistortion(new uberModel::DistortionFFReferentialA2W());
	g_sim->addDistortion(new uberModel::DistortionFFReferentialW2A());
	g_sim->addDistortion(new uberModel::DistortionSpeed);
	g_sim->addDistortion(new uberModel::DistortionSpeedUncertainty);
	g_sim->addDistortion(new uberModel::DistortionRadius);
	g_sim->addDistortion(new uberModel::DistortionTimeUncertainty);
	g_sim->addDistortion(new uberModel::DistortionTHorizon());
	g_sim->initParams();
	g_sim->setDistribution(new uberModel::DistributionGaussian);
// 	g_sim->setDistribution(new uberModel::DistributionSolid);
	
// 	if (g_params) delete [] g_params;
// 	g_params = g_sim->params();
}

void SimulationUber::reset()
{
	if (g_navigator) g_navigator->reset();
// 	g_sim->reset();
// 	memcpy(g_sim->params(), g_params, g_nPedestrian*g_nbParams*sizeof(float));
	
	for (int i = 0; i < g_nPedestrian; i++) {
		g_sim->params()[i*g_sim->getNParams() + 0] = g_params[i*8 + 0];
		g_sim->params()[i*g_sim->getNParams() + 2] = g_params[i*8 + 1];
		g_sim->params()[i*g_sim->getNParams() + 4] = g_params[i*8 + 2];
		g_sim->params()[i*g_sim->getNParams() + 5] = g_params[i*8 + 3];
		g_sim->params()[i*g_sim->getNParams() + 6] = g_params[i*8 + 4];
		g_sim->params()[i*g_sim->getNParams() + 7] = g_params[i*8 + 5];
		g_sim->params()[i*g_sim->getNParams() + 8] = g_params[i*8 + 6];
		g_sim->params()[i*g_sim->getNParams() + 9] = g_params[i*8 + 7];
	}
}

void SimulationUber::setPosition(int s_indPedestrian, float s_x, float s_y)
{
	g_sim->currentAgentState(s_indPedestrian)[0] = s_x;
	g_sim->currentAgentState(s_indPedestrian)[1] = s_y;
}

void SimulationUber::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
	g_sim->currentAgentState(s_indPedestrian)[2] = s_x;
	g_sim->currentAgentState(s_indPedestrian)[3] = s_y;
}

void SimulationUber::setGoal(int s_indPedestrian, float s_x, float s_y)
{
// 	g_sim->goal(s_indPedestrian)[0] = s_x;
// 	g_sim->goal(s_indPedestrian)[1] = s_y;
	if (s_x != g_gx[s_indPedestrian] || s_y != g_gy[s_indPedestrian]) g_reloadGoals = true;
	
	g_gx[s_indPedestrian] = s_x;
	g_gy[s_indPedestrian] = s_y;
}

void SimulationUber::doStep(float s_dt)
{
	if (g_reloadGoals) update();
	if (g_navigator) {
		for (int i = 0; i < g_nPedestrian; i++) {
			g_navigator->navigate(
				i,
				g_sim->goal(i)[0],
				g_sim->goal(i)[1],
				g_sim->currentAgentState(i)[0],
				g_sim->currentAgentState(i)[1],
				g_sim->getParam(i, 2)
			);
		}
	}
	
	g_sim->update();
	g_sim->doStep(s_dt, 10);
	g_sim->advance();
	
	for (int i = 0; i < g_nPedestrian; i++) {
		setNextState(i, g_sim->currentAgentState(i)[0], g_sim->currentAgentState(i)[1], g_sim->currentAgentState(i)[2], g_sim->currentAgentState(i)[3]);
	}
}

void SimulationUber::updateSpecific()
{
	if (
		g_reloadGoals ||
		changed("Use environment") ||
		(changed("Environment file") && g_useEnvironment)
	) {
		uberModel::Path * path;
		if (g_useEnvironment) {
			std::cout<< "env starting\n";
			
			uberModel::CurveNode::reset();
			std::cout<< "curves reset\n";
			if (g_navigator) delete g_navigator;
			g_navigator = new uberModel::Navigator();
			std::cout<< "initting navigation "<< getConfigurableValue("Environment file").c_str()<< "\n";
			g_navigator->loadEnvironment(getConfigurableValue("Environment file").c_str());
			g_navigator->setNAgents(g_nPedestrian);
			
			std::cout<< "env initted\n";
			
			for (int i = 0; i < g_nPedestrian; i++) {
				g_navigator->setAgent(
					i,
					g_sim->currentAgentState(i)[0],
					g_sim->currentAgentState(i)[1],
					g_gx[i],
					g_gy[i]
				);
				
				
// 				g_nav->reset();
// 				g_nav->setStart(
// 					g_sim->currentAgentState(i)[0],
// 					g_sim->currentAgentState(i)[1]
// 				);
// 				g_nav->setEnd(g_gx[i], g_gy[i]);
// 				std::cout<< "set "<< g_sim->currentAgentState(i)[0]<< " "<< g_sim->currentAgentState(i)[1];
// 				std::cout<< " "<< g_gx[i]<< " "<< g_gy[i]<< std::endl;
// 				path = g_nav->findPath();
// // 				std::cout<< path<< std::endl;
// // 				path->print();
// 				
// 				g_sim->setGoal(i, path);
			}
			
			std::cout<< "env ok\n";
		}
		else {
			for (int i = 0; i < g_nPedestrian; i++) {
// 				path = new uberModel::Path;
// 				path->addNode(new uberModel::Path::SimpleNode(g_gx[i], g_gy[i], 1.0));
// 				g_sim->setGoal(i, path);
				g_sim->goal(i)[0] = g_gx[i];
				g_sim->goal(i)[1] = g_gy[i];
			}
		}
	}
	
	g_reloadGoals = false;
}

}
#endif
