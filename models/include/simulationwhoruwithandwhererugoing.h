#ifndef _SIMULATIONWHORUWITHANDWHERERUGOING_H
#define _SIMULATIONWHORUWITHANDWHERERUGOING_H

#include "Craal/core/include/Simulation.h"
#include "DEFINITIONS.h"
#include <QVector2D>
#include <QVector>

namespace craal
{

class SimulationWhoRUwithAndWhereRUgoing : public Simulation
{
public:
    SimulationWhoRUwithAndWhereRUgoing();

    class Agent{
    public:
        unsigned ID;
        QVector2D pos;
        QVector2D vel;
        float radius;
        QVector2D nextVel;
        QVector<Agent*> neighbors;
        QVector<QLineF> nearObstacle;

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
    void setObstacles(int index);

public:
    virtual void init();
    virtual void reset();

    virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);

    virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
    virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
    virtual void setGoal(int s_indPedestrian, float s_x, float s_y);
    virtual void doStep(float s_dt = 0.01);

    void setPreferredVelocity(int s_indPedestrian, float s_x, float s_y);

private:
    const float epsilon = 1E-6;
    const float diff_h  = 1E-6;
    const float gama = 0.05;
    // parameters
    float lambda[6];
    float sig_w, sig_d, beta_fov;


    float DCA (const QVector2D &deltaP, const QVector2D &deltaV);

    QVector2D socialForceYamaguchi(int index_i);
    QVector2D socialForceAlexandre(int index_i);

    QVector2D EnergyGradFixedObstacle(int index);

    QVector2D EnergyGradDamping(int index);
    QVector2D EnergyGradSpeed(int index);
    QVector2D EnergyGradDirection(int index);
    QVector2D EnergyGradAttraction(int index);
    QVector2D EnergyGradGroup(int index);
    QVector2D EnergyGradCollision(int index);

    void update(int index, float dt);


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
#endif // SIMULATIONWHORUWITHANDWHERERUGOING_H
