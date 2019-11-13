#include "../include/SimulationTModel.h"

namespace craal {

SimulationTModel::SimulationTModel()
{
    addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
    addParam("Radius", 0.25, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
    addParam("Beta", 0.5, PDF(0.0, 1.0, PDF::NORMAL, 0.5, 0.5));
    addParam("Gamma", 0.25, PDF(0.0, 0.6, PDF::NORMAL, 0.3, 0.3));

    name = "T-Model";
}

SimulationTModel::~SimulationTModel()
{
    p_deleteAgents();
}

void SimulationTModel::p_deleteAgents()
{
    for (int i = 0; i < g_agents.size(); i++) {
        delete g_agents[i];
    }
    SCA09::Agent::agents.clear();
    g_agents.clear();
}

void SimulationTModel::init()
{
    p_deleteAgents();

    for (int i = 0; i < g_nPedestrian; i++) {
        g_agents.push_back(new SCA09::Agent());
        g_agents[i]->init();
    }
}

void SimulationTModel::reset()
{
    for (int i = 0; i < g_nPedestrian; i++) {
        g_agents[i]->speedComfort = g_params[i*4+0];
        g_agents[i]->personalArea = g_params[i*4+1];
        g_agents[i]->g_beta = g_params[i*4+2];
        g_agents[i]->g_gamma = g_params[i*4+3];
    }
}

void SimulationTModel::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    Wm5::Vector2f vec1(s_startx, s_starty);
    Wm5::Vector2f vec2(s_endx, s_endy);
    Wm5::Segment2f seg(vec1, vec2);

    SCA09::Agent::addObstacle(seg);
}

void SimulationTModel::setPosition(int s_indPedestrian, float s_x, float s_y)
{
    g_agents[s_indPedestrian]->position = Wm5::Vector2f(s_x, s_y);
}

void SimulationTModel::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
    Wm5::Vector2f vec(s_x, s_y);
    float speed = vec.Length();
    vec.Normalize();

    g_agents[s_indPedestrian]->direction = vec;
    g_agents[s_indPedestrian]->speed = speed;
}

void SimulationTModel::setGoal(int s_indPedestrian, float s_x, float s_y)
{
    g_agents[s_indPedestrian]->goal = Wm5::Vector2f(s_x, s_y);
}

void SimulationTModel::doStep(float s_dt)
{
    Wm5::Vector2f vec1, vec2;

    for (int i = 0; i < g_nPedestrian; i++) {
        g_agents[i]->updateVelocity(s_dt);
        g_agents[i]->updatePosition(s_dt);

        vec1 = g_agents[i]->position;
        vec2 = g_agents[i]->direction * g_agents[i]->speed;

        setNextState(i, vec1.X(), vec1.Y(), vec2.X(), vec2.Y());
    }
}

}
