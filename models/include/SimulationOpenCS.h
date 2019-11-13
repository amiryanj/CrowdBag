//========================================================================
/*!
  @file		SimulationOpenCS.h
  @class	SimulationOpenCS
  @date    	08/6/2018
  @brief
  @author  Javad Amirian, (C) 2018
*/
//========================================================================


#ifndef _SIMULATIONOPENCS_H
#define _SIMULATIONOPENCS_H

#include "core/include/Simulation.h"
#include <QVector2D>
#include <QVector>
#include <QLineF>
#include <cmath>

namespace craal {

class SimulationOpenCS : public Simulation
{
public:
    SimulationOpenCS();
    ~SimulationOpenCS();

friend class World;
public:
    class Agent{
    public:
        unsigned ID;
        QVector2D pos;
        QVector2D vel;
        float radius;        
        QVector2D nextVel;
        QVector<Agent*> neighbors;
        QLineF nearObstacle[2];

        QVector2D tmp_ttca_vec;
        QVector2D tmp_dca_vec;
        QVector2D tmp_rep_vec;
        QVector2D tmp_goal_vec;
    };

    QVector<Agent*> agentlist;
    QVector<QLineF> obstacleList;

    float *g_gx = NULL, *g_gy = NULL;
    float timeStep;

    void p_delete2();
    void computeNeighbors();
    void setNearObstacles(int index);

public:
    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y);

    virtual void doStep(float s_dt = 0.01);

private:
    bool debug = false;
    float epsilon = 1E-3;
    float randRange = 0.001f;
    float beta_fov = 1.;
    QVector2D randomVect ;
    // parameters
    float SigTTCA = 1.5, SigDCA = 1.5, SigPS = 20;

    float TTCA(const QVector2D &deltaP, const QVector2D &deltaV);
    float DCA (const QVector2D &deltaP, const QVector2D &deltaV);

    QVector2D TTCA_Grad(const QVector2D &deltaP, const QVector2D &deltaV);
    QVector2D DCA_Grad (const QVector2D &deltaP, const QVector2D &deltaV);

    QVector2D socialForceFabien(int index_i);
    QVector2D socialForceAlexandre(int index_i);

    QVector2D goalForceFabien(int index);
    QVector2D goalForceYamaguchi(int index);
    QVector2D fixedObstacleForce(int index);

    void FixedUpdate(int index, float dt);


    inline float sigmoid(float x, float sigma) {
        return tanh(sigma*x);
    }
    inline float expCost(double value, double sigma) {
        return exp(-0.5 * fabs(value/sigma));
    }
    inline float gaussianCost(double value, double sigma) {
        return exp(-0.5 * pow(value/sigma, 2.));
    }
    inline float logCost(double value, double sigma) {
        return -log(1+fabs(value)/sigma);
    }

};

}

#endif // SIMULATIONOPENCS_H
