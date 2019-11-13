#include "SimulationRVO2-NAV.h"
#ifdef UBER
namespace craal {

SimulationRVO2NAV::SimulationRVO2NAV() :
	g_sim(0), g_gx(0), g_gy(0), g_useEnvironment(false),
	g_envFile("None"), g_reloadGoals(false), g_nav(0), g_paths(0)
{
// 	addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
// 	addParam("Neighbor dist", 15.0, PDF(10, 20, PDF::NORMAL, 15, 5));
// 	addParam("Radius", 0.25, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
// 	addParam("Agent t.h.", 5, PDF(0.1, 5, PDF::NORMAL, 2, 2));
// 	addParam("Obstacle t.h.", 5, PDF(0.1, 5, PDF::NORMAL, 2, 2));
	
	addParam("Speed", 1.4, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
	addParam("Neighbor dist", 15.0, PDF(10, 20, PDF::NORMAL, 15, 5));
	addParam("Radius", 0.5, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
	addParam("Agent t.h.", 5, PDF(3, 7, PDF::NORMAL, 5, 2));
	addParam("Obstacle t.h.", 5, PDF(3, 7, PDF::NORMAL, 5, 2));
	
	addConfigurable("Use environment", new Type_bool, &g_useEnvironment);
	addConfigurable("Environment file", new Type_string, &g_envFile);
	
	name = "RVO2-NAV";
}

SimulationRVO2NAV::~SimulationRVO2NAV()
{
	p_delete2();
}

void SimulationRVO2NAV::p_delete2()
{
	if (g_sim) delete g_sim;
	if (g_gx) delete [] g_gx;
	if (g_gy) delete [] g_gy;
	g_sim = 0;
	g_gx = 0;
	g_gy = 0;
	if (g_nav) delete g_nav;
	g_nav = 0;
	if (g_paths) {
		for (int i = 0; i < g_nPedestrian; i++) {
			if (g_paths[i]) delete g_paths[i];
		}
		
		delete [] g_paths;
	}
	g_paths = 0;
}

void SimulationRVO2NAV::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
	std::vector<RVO::Vector2> obs;
	
	obs.push_back(RVO::Vector2(s_startx, s_starty));
	obs.push_back(RVO::Vector2(s_endx, s_endy));
	
	g_sim->addObstacle(obs);
}

void SimulationRVO2NAV::init()
{
	p_delete2();
	
	g_sim = new RVO::RVOSimulator();
	g_gx = new float[g_nPedestrian];
	g_gy = new float[g_nPedestrian];
	g_paths = new uberModel::Path*[g_nPedestrian];
	for (int i = 0; i < g_nPedestrian; i++) {
		g_paths[i] = 0;
	}
	
	g_sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 0.25f, 1.6f);
	
	for (int i = 0; i < g_nPedestrian; i++) {
		g_sim->addAgent(RVO::Vector2(0, 0));
	}
}

void SimulationRVO2NAV::reset()
{
	for (int i = 0; i < g_nPedestrian; i++) {
		g_sim->setAgentMaxSpeed(i, g_params[i*5+0]);
		g_sim->setAgentNeighborDist(i, g_params[i*5+1]);
		g_sim->setAgentRadius(i, g_params[i*5+2]);
		g_sim->setAgentTimeHorizon(i, g_params[i*5+3]);
		g_sim->setAgentTimeHorizonObst(i, g_params[i*5+4]);
	}
	
	g_sim->processObstacles();
	
	for (int i = 0; i < g_nPedestrian; i++) {
		if (g_paths[i]) g_paths[i]->reset();
	}
}

void SimulationRVO2NAV::setPosition(int s_indPedestrian, float s_x, float s_y)
{
	g_sim->setAgentPosition(s_indPedestrian, RVO::Vector2(s_x, s_y));
}

void SimulationRVO2NAV::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
	g_sim->setAgentVelocity(s_indPedestrian, RVO::Vector2(s_x, s_y));
}

void SimulationRVO2NAV::setGoal(int s_indPedestrian, float s_x, float s_y)
{
	if (s_x != g_gx[s_indPedestrian] || s_y != g_gy[s_indPedestrian]) g_reloadGoals = true;
	
	g_gx[s_indPedestrian] = s_x;
	g_gy[s_indPedestrian] = s_y;
}

void SimulationRVO2NAV::doStep(float s_dt)
{
	if (g_reloadGoals) update();
	
	RVO::Vector2 vec;
// 	float tmp;
	
	g_sim->setTimeStep(s_dt);
	for (int i = 0; i < g_nPedestrian; i++) {
		vec = g_sim->getAgentPosition(i);
		
// 		tmp = g_sim->getAgentVelocity(i).x()*g_sim->getAgentVelocity(i).x();
// 		tmp += g_sim->getAgentVelocity(i).y()*g_sim->getAgentVelocity(i).y();
		
		g_paths[i]->update(g_gx[i],
			g_gy[i],
			g_sim->getAgentPosition(i).x(),
			g_sim->getAgentPosition(i).y(),
  			g_sim->getAgentMaxSpeed(i));
		
		vec = RVO::Vector2(g_gx[i], g_gy[i])-vec;
		if (RVO::absSq(vec) > g_params[i*5+0]*g_params[i*5+0]) {
			vec = RVO::normalize(vec);
			vec = vec*g_params[i*5+0];
		}
		g_sim->setAgentPrefVelocity(i, vec);
	}
	g_sim->doStep();
	
	for (int i = 0; i < g_nPedestrian; i++) {
		setNextState(i, g_sim->getAgentPosition(i).x(), g_sim->getAgentPosition(i).y(), g_sim->getAgentVelocity(i).x(), g_sim->getAgentVelocity(i).y());
	}
}

void SimulationRVO2NAV::updateSpecific()
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
			if (g_nav) delete g_nav;
			g_nav = new uberModel::Navigation();
			std::cout<< "initting navigation "<< getConfigurableValue("Environment file").c_str()<< "\n";
			g_nav->init(getConfigurableValue("Environment file").c_str());
			
			std::cout<< "env initted\n";
			
			for (int i = 0; i < g_nPedestrian; i++) {
				g_nav->reset();
				g_nav->setStart(
					g_sim->getAgentPosition(i).x(),
					g_sim->getAgentPosition(i).y()
				);
				g_nav->setEnd(g_gx[i], g_gy[i]);
				std::cout<< "set "<< g_sim->getAgentPosition(i).x()<< " "<< g_sim->getAgentPosition(i).y();
				std::cout<< " "<< g_gx[i]<< " "<< g_gy[i]<< std::endl;
				path = g_nav->findPath();
// 				std::cout<< path<< std::endl;
// 				path->print();
				
				g_paths[i] = path;
			}
			
			std::cout<< "env ok\n";
		}
		else {
			for (int i = 0; i < g_nPedestrian; i++) {
				path = new uberModel::Path;
				path->addNode(new uberModel::Path::SimpleNode(g_gx[i], g_gy[i], 1.0));
				g_paths[i] = path;
			}
		}
	}
	
	g_reloadGoals = false;
}

}
#endif
