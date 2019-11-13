#include "../include/Experiment3D.h"

namespace craal {

Experiment3D::Experiment3D() :
	g_centerz(0), g_centerVelocityz(0), g_waypointz(0)
{}

Experiment3D::Experiment3D(Parser3D & s_parser) :
	g_centerz(0), g_centerVelocityz(0), g_waypointz(0)
{
	init(s_parser);
}

Experiment3D::~Experiment3D()
{
	if (g_centerz) delete [] g_centerz;
	if (g_centerVelocityz) delete [] g_centerVelocityz;
	if (g_waypointz) {
		for (int i = 0; i < g_nPedestrian; i++) {
			if (g_waypointz[i]) delete [] g_waypointz[i];
		}
		delete [] g_waypointz;
	}
}

void Experiment3D::init(Parser3D & s_parser)
{
	s_parser.fill(&g_nPedestrian, &g_nObstacle, &g_nMeasure,
		&g_centerx, &g_centery, &g_centerz,
		&g_centerVelocityx, &g_centerVelocityy, &g_centerVelocityz,
		&g_obstaclesx, &g_obstaclesy, &g_obstacleex, &g_obstacleey,
		&g_times,
		&g_realData, &g_nWaypoint, &g_waypointSize,
		&g_waypointx, &g_waypointy, &g_waypointz,
		&g_onlySimulateNb, &g_onlySimulate);
}

}
