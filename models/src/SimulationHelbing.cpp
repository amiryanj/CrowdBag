#include "../include/SimulationHelbing.h"

namespace craal {

SimulationHelbing::SimulationHelbing() :
    g_sim(0), g_gx(0), g_gy(0)
{
    // 	addParam("Radius", 0.2, PDF(0.2, 0.5, PDF::NORMAL, 0.35, 0.15));
    // 	addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));

    addParam("Radius", 0.2, PDF(0.1, 1, PDF::NORMAL, 0.3, 0.2));
    addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));

    name = "Helbing";
}

SimulationHelbing::~SimulationHelbing()
{
    p_delete2();
}

void SimulationHelbing::p_delete2()
{
    if (g_sim) delete g_sim;
    if (g_gx) delete [] g_gx;
    if (g_gy) delete [] g_gy;
    g_sim = 0;
    g_gx = 0;
    g_gy = 0;
}

void SimulationHelbing::init()
{
    p_delete2();

    g_sim = new helbing::HelbingSim();
    g_gx = new float[g_nPedestrian];
    g_gy = new float[g_nPedestrian];

    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->addAgent(0, 0, 0, 0);
        g_params[i*2+0] = 0.2;
        g_params[i*2+1] = 1.5;
        g_sim->setRadius(i, 0.2);
    }

    g_sim->doBoids = false;
    g_sim->init();
}

void SimulationHelbing::reset()
{
    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->setRadius(i, g_params[i*2+0]);
    }
}

void SimulationHelbing::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    g_sim->addObstacle(s_startx, s_starty, s_endx, s_endy);
}

void SimulationHelbing::setPosition(int s_indPedestrian, float s_x, float s_y)
{
    g_sim->setPos(s_indPedestrian, s_x, s_y);
}

void SimulationHelbing::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
    g_sim->setVel(s_indPedestrian, s_x, s_y);
}

void SimulationHelbing::setGoal(int s_indPedestrian, float s_x, float s_y)
{
    g_gx[s_indPedestrian] = s_x;
    g_gy[s_indPedestrian] = s_y;
}

void SimulationHelbing::doStep(float s_dt)
{
    g_sim->setTimeStep(s_dt);

    float x, y, pref_speed;
    for (int i = 0; i < g_nPedestrian; i++) {
        x = g_sim->getAgent(i).pos.x;
        y = g_sim->getAgent(i).pos.y;

        x = g_gx[i]-x;
        y = g_gy[i]-y;

        pref_speed = g_params[i*2+1];

        if (x*x+y*y > pref_speed*pref_speed)
//        if(x*x+y*y > 0.0000001)
        {
            pref_speed = sqrt(x*x+y*y);
            x /= pref_speed;
            y /= pref_speed;
            x *= g_params[i*2+1];
            y *= g_params[i*2+1];
        }
        g_sim->setGoalVel(i, x, y);
    }
    g_sim->doStep();
    for (int i = 0; i < g_nPedestrian; i++) {
        setNextState(i, g_sim->getAgent(i).pos.x, g_sim->getAgent(i).pos.y, g_sim->getAgent(i).vel.x, g_sim->getAgent(i).vel.y);
    }
}

void SimulationHelbing::setAgentRadius(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 2 + 0] = value;
    g_sim->setRadius(s_indPedestrian, g_params[s_indPedestrian*2+0]);

}

void SimulationHelbing::setAgentSpeed(int s_indPedestrian, float value)
{
    g_params[s_indPedestrian * 2 + 1] = value;
}

}
