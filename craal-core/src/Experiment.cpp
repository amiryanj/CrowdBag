#include "../include/Experiment.h"

namespace craal {

Experiment::Experiment() :
	g_time(0), g_nPedestrian(0), g_nObstacle(0), g_nMeasure(0),g_centerx(0), g_centery(0), g_centerVelocityx(0), g_centerVelocityy(0), g_times(0), g_obstaclesx(0), g_obstaclesy(0), g_obstacleex(0), g_obstacleey(0), g_realData(true), g_nWaypoint(0), g_waypointSize(0), g_waypointx(0), g_waypointy(0), g_onlySimulateNb(0), g_onlySimulate(0)
{}

Experiment::Experiment(Parser & s_parser) :
	g_time(0), g_nPedestrian(0), g_nObstacle(0), g_nMeasure(0), g_centerx(0), g_centery(0), g_centerVelocityx(0), g_centerVelocityy(0), g_times(0), g_obstaclesx(0), g_obstaclesy(0), g_obstacleex(0), g_obstacleey(0), g_realData(true), g_nWaypoint(0), g_waypointSize(0), g_waypointx(0), g_waypointy(0), g_onlySimulateNb(0), g_onlySimulate(0)
{
	init(s_parser);
}

Experiment::~Experiment()
{
	if (g_centerx) delete [] g_centerx;
	if (g_centery) delete [] g_centery;
	if (g_centerVelocityx) delete [] g_centerVelocityx;
	if (g_centerVelocityy) delete [] g_centerVelocityy;
	if (g_times) delete [] g_times;
	if (g_obstaclesx) delete [] g_obstaclesx;
	if (g_obstaclesy) delete [] g_obstaclesy;
	if (g_obstacleex) delete [] g_obstacleex;
	if (g_obstacleey) delete [] g_obstacleey;
	if (g_nWaypoint) delete [] g_nWaypoint;
	if (g_onlySimulate) delete [] g_onlySimulate;
	if (g_waypointx) {
		for (int i = 0; i < g_nPedestrian; i++) {
			if (g_waypointx[i]) delete [] g_waypointx[i];
		}
		delete [] g_waypointx;
	}
	if (g_waypointy) {
		for (int i = 0; i < g_nPedestrian; i++) {
			if (g_waypointy[i]) delete [] g_waypointy[i];
		}
		delete [] g_waypointy;
	}
	if (g_waypointSize) {
		for (int i = 0; i < g_nPedestrian; i++) {
			if (g_waypointSize[i]) delete [] g_waypointSize[i];
		}
		delete [] g_waypointSize;
	}
}

void Experiment::init(Parser & s_parser)
{
	s_parser.fill(&g_nPedestrian, &g_nObstacle, &g_nMeasure,
		&g_centerx, &g_centery,
		&g_centerVelocityx, &g_centerVelocityy,
		&g_obstaclesx, &g_obstaclesy, &g_obstacleex, &g_obstacleey,
		&g_times,
		&g_realData, &g_nWaypoint, &g_waypointSize, &g_waypointx, &g_waypointy,
		&g_onlySimulateNb, &g_onlySimulate);
}

}
