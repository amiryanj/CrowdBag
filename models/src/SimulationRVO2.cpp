#include "../include/SimulationRVO2.h"

namespace craal {

SimulationRVO2::SimulationRVO2() :
    g_sim(0), g_gx(0), g_gy(0)
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
    addParam("Obstacle t.h.", 5, PDF(3, 7, PDF::NORMAL, 5, 2));

    name = "RVO2";
}

SimulationRVO2::~SimulationRVO2()
{
    p_delete2();
}

void SimulationRVO2::p_delete2()
{
    if (g_sim) delete g_sim;
    if (g_gx) delete [] g_gx;
    if (g_gy) delete [] g_gy;
    g_sim = 0;
    g_gx = 0;
    g_gy = 0;
}

void SimulationRVO2::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    std::vector<RVO::Vector2> obs;

    obs.push_back(RVO::Vector2(s_startx, s_starty));
    obs.push_back(RVO::Vector2(s_endx, s_endy));

    g_sim->addObstacle(obs);
}

void SimulationRVO2::init()
{
    p_delete2();

    g_sim = new RVO::RVOSimulator();
    g_gx = new float[g_nPedestrian];
    g_gy = new float[g_nPedestrian];

    g_sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 0.25f, 1.6f);

    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->addAgent(RVO::Vector2(0, 0));
    }
}

void SimulationRVO2::reset()
{
    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->setAgentMaxSpeed(i, g_params[i*5+0]);
        g_sim->setAgentNeighborDist(i, g_params[i*5+1]);
        g_sim->setAgentRadius(i, g_params[i*5+2]);
        g_sim->setAgentTimeHorizon(i, g_params[i*5+3]);
        g_sim->setAgentTimeHorizonObst(i, g_params[i*5+4]);
    }

    g_sim->processObstacles();
}

void SimulationRVO2::setPosition(int s_indPedestrian, float s_x, float s_y)
{
    g_sim->setAgentPosition(s_indPedestrian, RVO::Vector2(s_x, s_y));
}

void SimulationRVO2::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
    g_sim->setAgentVelocity(s_indPedestrian, RVO::Vector2(s_x, s_y));
}

void SimulationRVO2::setGoal(int s_indPedestrian, float s_x, float s_y)
{
    g_gx[s_indPedestrian] = s_x;
    g_gy[s_indPedestrian] = s_y;
}

void SimulationRVO2::doStep(float s_dt)
{
    RVO::Vector2 vec;

    g_sim->setTimeStep(s_dt);
    for (int i = 0; i < g_nPedestrian; i++) {
        vec = g_sim->getAgentPosition(i);

        vec = RVO::Vector2(g_gx[i], g_gy[i])-vec;
        //if (RVO::absSq(vec) > g_params[i*5+0]*g_params[i*5+0])
        if(RVO::absSq(vec) > 0.0000001)
        {
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

void SimulationRVO2::setAgentMaxSpeed(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 5 + 0] = value;
    g_sim->setAgentMaxSpeed(s_indPedestrian, g_params[s_indPedestrian*5+0]);
}

void SimulationRVO2::setAgentNeighborDist(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 5 + 1] = value;
    g_sim->setAgentNeighborDist(s_indPedestrian, g_params[s_indPedestrian*5+1]);
}

void SimulationRVO2::setAgentRadius(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 5 + 2] = value;
    g_sim->setAgentRadius(s_indPedestrian, g_params[s_indPedestrian*5+2]);
}

void SimulationRVO2::setAgentTimeHorizon(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 5 + 3] = value;
    g_sim->setAgentTimeHorizon(s_indPedestrian, g_params[s_indPedestrian*5+3]);
}

void SimulationRVO2::setAgentTimeHorizonObst(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 5 + 4] = value;
    g_sim->setAgentTimeHorizonObst(s_indPedestrian, g_params[s_indPedestrian*5+4]);
}

}
