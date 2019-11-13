#include "../include/Simulation3D.h"

namespace craal {

Simulation3D::Simulation3D() :
	g_centerz(0),
	g_centerVelocityz(0),
	g_centerzNext(0),
	g_centerVelocityzNext(0)
{}

Simulation3D::~Simulation3D()
{
	p_deleteArrays();
}

void Simulation3D::p_deleteArrays()
{
	if (g_centerz) delete [] g_centerz;
	if (g_centerVelocityz) delete [] g_centerVelocityz;
	if (g_centerzNext) delete [] g_centerzNext;
	if (g_centerVelocityzNext) delete [] g_centerVelocityzNext;
	
	g_centerz = 0;
	g_centerVelocityz = 0;
	g_centerzNext = 0;
	g_centerVelocityzNext = 0;
}

void Simulation3D::initSimulation(int s_nPedestrian)
{
	Simulation::p_deleteArrays();
	p_deleteArrays();
	g_nPedestrian = s_nPedestrian;
	
	g_centerx = new std::vector<float>[s_nPedestrian];
	g_centery = new std::vector<float>[s_nPedestrian];
	g_centerz = new std::vector<float>[s_nPedestrian];
	g_centerVelocityx = new std::vector<float>[s_nPedestrian];
	g_centerVelocityy = new std::vector<float>[s_nPedestrian];
	g_centerVelocityz = new std::vector<float>[s_nPedestrian];
	
	g_centerxNext = new float[s_nPedestrian];
	g_centeryNext = new float[s_nPedestrian];
	g_centerzNext = new float[s_nPedestrian];
	g_centerVelocityxNext = new float[s_nPedestrian];
	g_centerVelocityyNext = new float[s_nPedestrian];
	g_centerVelocityzNext = new float[s_nPedestrian];
	
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

void Simulation3D::addState()
{
	for (int i = 0; i < g_nPedestrian; i++) {
		g_centerx[i].push_back(g_centerxNext[i]);
		g_centery[i].push_back(g_centeryNext[i]);
		g_centerz[i].push_back(g_centerzNext[i]);
		g_centerVelocityx[i].push_back(g_centerVelocityxNext[i]);
		g_centerVelocityy[i].push_back(g_centerVelocityyNext[i]);
		g_centerVelocityz[i].push_back(g_centerVelocityzNext[i]);
	}
}

void Simulation3D::addState(int s_indPedestrian,
	float s_px, float s_py, float s_pz,
	float s_vx, float s_vy, float s_vz)
{
	g_centerx[s_indPedestrian].push_back(s_px);
	g_centery[s_indPedestrian].push_back(s_py);
	g_centerz[s_indPedestrian].push_back(s_pz);
	g_centerVelocityx[s_indPedestrian].push_back(s_vx);
	g_centerVelocityy[s_indPedestrian].push_back(s_vy);
	g_centerVelocityz[s_indPedestrian].push_back(s_vz);
}

void Simulation3D::setNextState(int s_indPedestrian,
	float s_px, float s_py, float s_pz,
	float s_vx, float s_vy, float s_vz)
{
	g_centerxNext[s_indPedestrian] = s_px;
	g_centeryNext[s_indPedestrian] = s_py;
	g_centerzNext[s_indPedestrian] = s_pz;
	g_centerVelocityxNext[s_indPedestrian] = s_vx;
	g_centerVelocityyNext[s_indPedestrian] = s_vy;
	g_centerVelocityzNext[s_indPedestrian] = s_vz;
}

void Simulation3D::clear()
{
	for (int i = 0; i < g_nPedestrian; i++) {
		g_centerx[i].clear();
		g_centery[i].clear();
		g_centerz[i].clear();
		g_centerVelocityx[i].clear();
		g_centerVelocityy[i].clear();
		g_centerVelocityz[i].clear();
	}
}

void Simulation3D::resetNMeasure()
{
	g_nMeasure = g_centerx->size();
}

}
