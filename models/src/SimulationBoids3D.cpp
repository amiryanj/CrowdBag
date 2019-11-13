#include "../include/SimulationBoids3D.h"

namespace craal {

SimulationBoids3D::SimulationBoids3D() :
    g_sim(0), g_gx(0), g_gy(0), g_gz(0)
{
    // 	addParam("Radius", 0.2, PDF(0.2, 0.8, PDF::NORMAL, 0.5, 0.25));
    // 	addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));

    addParam("Radius", 0.2, PDF(0.1, 1, PDF::NORMAL, 0.3, 0.2));
    addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));

    name = "Boids3D";
}

SimulationBoids3D::~SimulationBoids3D()
{
    p_delete2();
}

void SimulationBoids3D::p_delete2()
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

void SimulationBoids3D::init()
{
    p_delete2();

    g_sim = new helbing3d::HelbingSim();
    g_gx = new float[g_nPedestrian];
    g_gy = new float[g_nPedestrian];
    g_gz = new float[g_nPedestrian];

    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->addAgent(0, 0, 0, 0, 0, 0);
        g_params[i*2+0] = 0.2;
        g_params[i*2+1] = 1.5;
        g_sim->setRadius(i, 0.2);
    }

    g_sim->doBoids = true;
    // 	g_sim->doBoids = false;
    g_sim->init();
}

void SimulationBoids3D::reset()
{
    for (int i = 0; i < g_nPedestrian; i++) {
        g_sim->setRadius(i, g_params[i*2+0]);
    }
}

void SimulationBoids3D::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    // 	g_sim->addObstacle(s_startx, s_starty, s_endx, s_endy);
}

void SimulationBoids3D::setPosition(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    g_sim->setPos(s_indPedestrian, s_x, s_y, s_z);
}

void SimulationBoids3D::setVelocity(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    g_sim->setVel(s_indPedestrian, s_x, s_y, s_z);
}

void SimulationBoids3D::setGoal(int s_indPedestrian, float s_x, float s_y, float s_z)
{
    g_gx[s_indPedestrian] = s_x;
    g_gy[s_indPedestrian] = s_y;
    g_gz[s_indPedestrian] = s_z;
}

void SimulationBoids3D::doStep(float s_dt)
{
    g_sim->setTimeStep(s_dt);

    float x, y, z, t, tmp;
    for (int i = 0; i < g_nPedestrian; i++) {
        x = g_sim->getAgent(i).pos.x;
        y = g_sim->getAgent(i).pos.y;
        z = g_sim->getAgent(i).pos.z;

        x = g_gx[i]-x;
        y = g_gy[i]-y;
        z = g_gz[i]-z;

        t = g_params[i*2+1];

        tmp = x*x+y*y+z*z;

        if (tmp > t*t) {
            t = sqrt(tmp);
            x /= t;
            y /= t;
            z /= t;
            x *= g_params[i*2+1];
            y *= g_params[i*2+1];
            z *= g_params[i*2+1];
        }
        g_sim->setGoalVel(i, x, y, z);
    }
    g_sim->doStep();
    for (int i = 0; i < g_nPedestrian; i++) {
        setNextState(
                    i,
                    g_sim->getAgent(i).pos.x,
                    g_sim->getAgent(i).pos.y,
                    g_sim->getAgent(i).pos.z,
                    g_sim->getAgent(i).vel.x,
                    g_sim->getAgent(i).vel.y,
                    g_sim->getAgent(i).vel.z
                    );
    }
}

}
