#include "../include/Simulation.h"

namespace craal {

Simulation::Simulation() :
	g_dt(0.01), g_time(0),
	g_nObstacle(0), g_nPedestrian(0), g_nMeasure(0), g_nbParams(0),
	g_centerx(0), g_centery(0),
	g_centerVelocityx(0), g_centerVelocityy(0), 
	g_centerxNext(0), g_centeryNext(0),
	g_centerVelocityxNext(0), g_centerVelocityyNext(0), g_params(0)
{}

Simulation::~Simulation()
{
	p_deleteArrays();
}

void Simulation::p_deleteArrays()
{
	if (g_centerx) delete [] g_centerx;
	if (g_centery) delete [] g_centery;
	if (g_centerVelocityx) delete [] g_centerVelocityx;
	if (g_centerVelocityy) delete [] g_centerVelocityy;
	if (g_centerxNext) delete [] g_centerxNext;
	if (g_centeryNext) delete [] g_centeryNext;
	if (g_centerVelocityxNext) delete [] g_centerVelocityxNext;
	if (g_centerVelocityyNext) delete [] g_centerVelocityyNext;
	if (g_params) delete [] g_params;
	
	g_centerx = 0;
	g_centery = 0;
	g_centerVelocityx = 0;
	g_centerVelocityy = 0;
	g_centerxNext = 0;
	g_centeryNext = 0;
	g_centerVelocityxNext = 0;
	g_centerVelocityyNext = 0;
	g_params = 0;
}

void Simulation::addParam(std::string s_name, float s_default, PDF s_pdf)
{
	g_defaults.push_back(s_default);
	g_pdfs.push_back(s_pdf);
	g_nbParams = g_defaults.size();
	
	std::vector<std::string> tmp = getConfigurables();
	
	clearConfigurables();
	for (int i = 0; i < g_pdfs.size()-1; i++) {
		addConfigurable(tmp[i], new Type_pdf, &(g_pdfs[i]));
	}
	addConfigurable(s_name, new Type_pdf, &(g_pdfs.back()));
}

void Simulation::initSimulation(int s_nPedestrian)
{
	p_deleteArrays();
	g_nPedestrian = s_nPedestrian;
	
	g_centerx = new std::vector<float>[s_nPedestrian];
	g_centery = new std::vector<float>[s_nPedestrian];
	g_centerVelocityx = new std::vector<float>[s_nPedestrian];
	g_centerVelocityy = new std::vector<float>[s_nPedestrian];
	
	g_centerxNext = new float[s_nPedestrian];
	g_centeryNext = new float[s_nPedestrian];
	g_centerVelocityxNext = new float[s_nPedestrian];
	g_centerVelocityyNext = new float[s_nPedestrian];
	
	g_params = new float[s_nPedestrian*g_nbParams];
// 	g_pdfs.clear();
	
	init();
	
	for (int i = 0; i < g_nPedestrian; i++) {
		for (int j = 0; j < g_nbParams; j++) {
			g_params[i*g_nbParams+j] = g_defaults[j];
		}
	}
	
	for (int i = 0; i < g_nObstacle; i++) {
		addObstacleCoords(g_obstaclesx[i], g_obstaclesy[i], g_obstacleex[i], g_obstacleey[i]);
	}
	
	reset();
}

void Simulation::addObstacle(float s_startx, float s_starty, float s_endx, float s_endy)
{
	g_obstaclesx.push_back(s_startx);
	g_obstaclesy.push_back(s_starty);
	g_obstacleex.push_back(s_endx);
	g_obstacleey.push_back(s_endy);
	g_nObstacle++;
}

void Simulation::addState()
{
	for (int i = 0; i < g_nPedestrian; i++) {
		g_centerx[i].push_back(g_centerxNext[i]);
		g_centery[i].push_back(g_centeryNext[i]);
		g_centerVelocityx[i].push_back(g_centerVelocityxNext[i]);
		g_centerVelocityy[i].push_back(g_centerVelocityyNext[i]);
	}
}

void Simulation::addState(int s_indPedestrian, float s_px, float s_py, float s_vx, float s_vy)
{
	g_centerx[s_indPedestrian].push_back(s_px);
	g_centery[s_indPedestrian].push_back(s_py);
	g_centerVelocityx[s_indPedestrian].push_back(s_vx);
	g_centerVelocityy[s_indPedestrian].push_back(s_vy);
}

void Simulation::setNextState(int s_indPedestrian, float s_px, float s_py, float s_vx, float s_vy)
{
	g_centerxNext[s_indPedestrian] = s_px;
	g_centeryNext[s_indPedestrian] = s_py;
	g_centerVelocityxNext[s_indPedestrian] = s_vx;
	g_centerVelocityyNext[s_indPedestrian] = s_vy;
}

void Simulation::clear()
{
	for (int i = 0; i < g_nPedestrian; i++) {
		g_centerx[i].clear();
		g_centery[i].clear();
		g_centerVelocityx[i].clear();
		g_centerVelocityy[i].clear();
	}
}

void Simulation::resetNMeasure()
{
	g_nMeasure = g_centerx->size();
}

}
