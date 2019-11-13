#include "../include/SimulationPowerLaw.h"

namespace craal {

SimulationPowerLaw::SimulationPowerLaw() :
    g_sim(0), g_gx(0), g_gy(0)
{
    addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
    addParam("Neighbor dist", 10.0, PDF(5, 20, PDF::NORMAL, 12, 7));
    addParam("Radius", 0.25, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
    addParam("Time horizon", 3, PDF(1, 7, PDF::NORMAL, 5, 2));
    addParam("k", 1.5, PDF(1, 3, PDF::NORMAL, 1.5, 1));
    addParam("ksi", 0.54, PDF(0.1, 2.5, PDF::NORMAL, 1, 1));
    addParam("m", 2.0, PDF(0.1, 4.0, PDF::NORMAL, 2, 2));
    addParam("Max acceleration", 20.0, PDF(1.0, 40.0, PDF::NORMAL, 20.0, 20.0));

    name = "PowerLaw";
}

SimulationPowerLaw::~SimulationPowerLaw()
{
    p_delete2();
}

void SimulationPowerLaw::p_delete2()
{
    if (g_sim) delete g_sim;
    if (g_gx) delete [] g_gx;
    if (g_gy) delete [] g_gy;
    g_sim = 0;
    g_gx = 0;
    g_gy = 0;
}

void SimulationPowerLaw::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    std::pair<TTC::Vector2D, TTC::Vector2D> obs;

    obs.first = TTC::Vector2D(s_startx, s_starty);
    obs.second = TTC::Vector2D(s_endx, s_endy);

    g_sim->addObstacle(obs);
}

void SimulationPowerLaw::init()
{
    p_delete2();

    g_sim = new TTC::SimulationEngine();
    g_gx = new float[g_nPedestrian];
    g_gy = new float[g_nPedestrian];

    g_sim->setMaxSteps(1000000000);
    g_sim->init(1000, 1000);

    TTC::AgentInitialParameters par;

    par.k = 1.5f;
    par.ksi = 0.54f;
    par.m = 2.0f;
    par.t0 = 3.f;
    par.neighborDist = 10.f;
    par.maxAccel = 20.f;
    par.radius = 0.3f;
    par.prefSpeed = 1.4f;
    par.goalRadius = 0.01f;

    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->addAgent(par);
    }
}

void SimulationPowerLaw::reset()
{
    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->getAgent(i)->_prefSpeed = g_params[i*8 + 0];
        g_sim->getAgent(i)->_neighborDist = g_params[i*8 + 1];
        g_sim->getAgent(i)->_radius = g_params[i*8 + 2];
        g_sim->getAgent(i)->_t0 = g_params[i*8 + 3];
        g_sim->getAgent(i)->_k = g_params[i*8 + 4];
        g_sim->getAgent(i)->_ksi = g_params[i*8 + 5];
        g_sim->getAgent(i)->_m = g_params[i*8 + 6];
        g_sim->getAgent(i)->_maxAccel = g_params[i*8 + 7];
    }
}

void SimulationPowerLaw::setPosition(int s_indPedestrian, float s_x, float s_y)
{
    g_sim->getAgent(s_indPedestrian)->_position.x = s_x;
    g_sim->getAgent(s_indPedestrian)->_position.y = s_y;
}

void SimulationPowerLaw::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
    g_sim->getAgent(s_indPedestrian)->_velocity.x = s_x;
    g_sim->getAgent(s_indPedestrian)->_velocity.y = s_y;
}

void SimulationPowerLaw::setGoal(int s_indPedestrian, float s_x, float s_y)
{
    g_gx[s_indPedestrian] = s_x;
    g_gy[s_indPedestrian] = s_y;
}

void SimulationPowerLaw::doStep(float s_dt)
{
    TTC::Vector2D vec;

    g_sim->setTimeStep(s_dt);
    for (int i = 0; i < g_nPedestrian; i++) {
        vec = g_sim->getAgent(i)->_position;

        vec = TTC::Vector2D(g_gx[i], g_gy[i]) - vec;
        //if (vec.lengthSqr() > g_sim->getAgent(i)->_prefSpeed*g_sim->getAgent(i)->_prefSpeed)
        if(vec.lengthSqr() > 0.0000001)
        {
            vec.normalize();
            vec = vec*g_sim->getAgent(i)->_prefSpeed;
        }
        g_sim->getAgent(i)->_vPref = vec;
    }
    g_sim->updateSimulation();

    for (int i = 0; i < g_nPedestrian; i++) {
        setNextState(i, g_sim->getAgent(i)->_position.x, g_sim->getAgent(i)->_position.y, g_sim->getAgent(i)->_velocity.x, g_sim->getAgent(i)->_velocity.y);
    }
}

void SimulationPowerLaw::setAgentPrefSpeed(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 8 + 0] = value;
    g_sim->getAgent(s_indPedestrian)->_prefSpeed = g_params[s_indPedestrian*8 + 0];
}

void SimulationPowerLaw::setAgentNeighborDist(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 8 + 1] = value;
    g_sim->getAgent(s_indPedestrian)->_neighborDist = g_params[s_indPedestrian*8 + 1];
}

void SimulationPowerLaw::setAgentRadius(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 8 + 2] = value;
    g_sim->getAgent(s_indPedestrian)->_radius = g_params[s_indPedestrian*8 + 2];
}

void SimulationPowerLaw::setAgentT0(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 8 + 3] = value;
    g_sim->getAgent(s_indPedestrian)->_t0 = g_params[s_indPedestrian*8 + 3];
}

void SimulationPowerLaw::setAgentK(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 8 + 4] = value;
    g_sim->getAgent(s_indPedestrian)->_k = g_params[s_indPedestrian*8 + 4];
}

void SimulationPowerLaw::setAgentKsi(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 8 + 5] = value;
    g_sim->getAgent(s_indPedestrian)->_ksi = g_params[s_indPedestrian*8 + 5];
}

void SimulationPowerLaw::setAgentM(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 8 + 6] = value;
    g_sim->getAgent(s_indPedestrian)->_m = g_params[s_indPedestrian*8 + 6];
}

void SimulationPowerLaw::setAgentMaxAcceleration(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 8 + 7] = value;
    g_sim->getAgent(s_indPedestrian)->_maxAccel = g_params[s_indPedestrian*8 + 7];
}

}
