#include "../include/SimulationRVO23D.h"

namespace craal {

SimulationRVO23D::SimulationRVO23D() :
    g_sim(0), g_gx(0), g_gy(0), g_gz(0)
{
    // 	addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
    // 	addParam("Neighbor dist", 15.0, PDF(10, 20, PDF::NORMAL, 15, 5));
    // 	addParam("Radius", 0.25, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
    // 	addParam("Agent t.h.", 5, PDF(0.1, 5, PDF::NORMAL, 2, 2));
    // 	addParam("Obstacle t.h.", 5, PDF(0.1, 5, PDF::NORMAL, 2, 2));

    addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
    addParam("Neighbor dist", 15.0, PDF(10, 20, PDF::NORMAL, 15, 5));
    addParam("Radius", 0.25, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
    addParam("Agent t.h.", 5, PDF(3, 7, PDF::NORMAL, 5, 2));
    // 	addParam("Obstacle t.h.", 5, PDF(3, 7, PDF::NORMAL, 5, 2));

    name = "RVO2-3D";
}

SimulationRVO23D::~SimulationRVO23D()
{
    p_delete2();
}

void SimulationRVO23D::p_delete2()
{
    if (g_sim) delete g_sim;
    if (g_gx) delete [] g_gx;
    if (g_gy) delete [] g_gy;
    if (g_gz) delete [] g_gz;
    g_sim = 0;
    g_gx = 0;
    g_gy = 0;
    g_gz = 0;
}

void SimulationRVO23D::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    // 	std::vector<RVO::Vector2> obs;
    //
    // 	obs.push_back(RVO::Vector2(s_startx, s_starty));
    // 	obs.push_back(RVO::Vector2(s_endx, s_endy));
    //
    // 	g_sim->addObstacle(obs);
}

void SimulationRVO23D::init()
{
    p_delete2();

    g_sim = new RVO3D::RVOSimulator();
    g_gx = new float[g_nPedestrian];
    g_gy = new float[g_nPedestrian];
    g_gz = new float[g_nPedestrian];

    g_sim->setAgentDefaults(15.0f, 10, 5.0f, 0.2f, 1.6f);

    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->addAgent(RVO3D::Vector3(0, 0, 0));
    }
}

void SimulationRVO23D::reset()
{
    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->setAgentMaxSpeed(i, g_params[i*4+0]);
        g_sim->setAgentNeighborDist(i, g_params[i*4+1]);
        g_sim->setAgentRadius(i, g_params[i*4+2]);
        g_sim->setAgentTimeHorizon(i, g_params[i*4+3]);
        // 		g_sim->setAgentTimeHorizonObst(i, g_params[i*5+4]);
    }

    // 	g_sim->processObstacles();
}

void SimulationRVO23D::setPosition(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    g_sim->setAgentPosition(s_indPedestrian, RVO3D::Vector3(s_x, s_y, s_z));
}

void SimulationRVO23D::setVelocity(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    g_sim->setAgentVelocity(s_indPedestrian, RVO3D::Vector3(s_x, s_y, s_z));
}

void SimulationRVO23D::setGoal(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    g_gx[s_indPedestrian] = s_x;
    g_gy[s_indPedestrian] = s_y;
    g_gz[s_indPedestrian] = s_z;
}

void SimulationRVO23D::doStep(float s_dt)
{
    RVO3D::Vector3 vec;

    g_sim->setTimeStep(s_dt);
    for (int i = 0; i < g_nPedestrian; i++) {
        vec = g_sim->getAgentPosition(i);

        vec = RVO3D::Vector3(g_gx[i], g_gy[i], g_gz[i])-vec;
        if (RVO3D::absSq(vec) > g_params[i*4+0]*g_params[i*4+0]) {
            vec = RVO3D::normalize(vec);
            vec = vec*g_params[i*4+0];
        }
        g_sim->setAgentPrefVelocity(i, vec);
    }
    g_sim->doStep();

    for (int i = 0; i < g_nPedestrian; i++) {
        setNextState(i,
                     g_sim->getAgentPosition(i).x(),
                     g_sim->getAgentPosition(i).y(),
                     g_sim->getAgentPosition(i).z(),
                     g_sim->getAgentVelocity(i).x(),
                     g_sim->getAgentVelocity(i).y(),
                     g_sim->getAgentVelocity(i).z());
    }
}

}
