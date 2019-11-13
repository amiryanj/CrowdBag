#include "../include/simulationwhoruwithandwhererugoing.h"
#include "geometrymath.h"

namespace craal {

SimulationWhoRUwithAndWhereRUgoing::SimulationWhoRUwithAndWhereRUgoing()
    :lambda{1.f, 1.f, 1.f, 0.f, 0.f, 1.f},
     sig_w(3.5), sig_d (1.2), beta_fov(1.)
{    

    name = "Yamaguchi";
}

void SimulationWhoRUwithAndWhereRUgoing::p_delete2()
{
    if (g_gx) delete [] g_gx;
    if (g_gy) delete [] g_gy;
    g_gx = 0;
    g_gy = 0;
    for(int i=0; i<agentlist.size(); i++)
        delete agentlist[i];
    agentlist.clear();
    obstacleList.clear();
}

void SimulationWhoRUwithAndWhereRUgoing::computeNeighbors()
{
    for(int i=0; i<agentlist.size(); i++)
        agentlist[i]->neighbors.clear();

    static float neighborhood_thre_sqr = pow(DEF_RADIUS * 15, 2);

    for(int i=0; i<g_nPedestrian; i++)
    {
        Agent* agent = agentlist[i];
        for(int j=0; j<i; j++)
        {
            if((agentlist[j]->pos - agent->pos).lengthSquared() < neighborhood_thre_sqr)
            {
                agent->neighbors.append(agentlist[j]);
                agentlist[j]->neighbors.append(agent);
            }
        }
    }
}

void SimulationWhoRUwithAndWhereRUgoing::init()
{
    p_delete2();

    g_gx = new float[g_nPedestrian];
    g_gy = new float[g_nPedestrian];

    agentlist.resize(g_nPedestrian);
    for(int i=0; i<g_nPedestrian; i++)
    {
        agentlist[i] = new Agent();
        agentlist[i]->radius = DEF_RADIUS;
        agentlist[i]->ID = i;
    }
}

void SimulationWhoRUwithAndWhereRUgoing::reset()
{

}

void SimulationWhoRUwithAndWhereRUgoing::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    QLineF obs;
    obs.setP1(QPointF(s_startx, s_starty));
    obs.setP2(QPointF(s_endx, s_endy));
    obstacleList.append(obs);
}

void SimulationWhoRUwithAndWhereRUgoing::setPosition(int s_indPedestrian, float s_x, float s_y)
{
    agentlist[s_indPedestrian]->pos.setX(s_x);
    agentlist[s_indPedestrian]->pos.setY(s_y);
}

void SimulationWhoRUwithAndWhereRUgoing::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
    agentlist[s_indPedestrian]->vel.setX(s_x);
    agentlist[s_indPedestrian]->vel.setY(s_y);
}

void SimulationWhoRUwithAndWhereRUgoing::setGoal(int s_indPedestrian, float s_x, float s_y)
{
    g_gx[s_indPedestrian] = s_x;
    g_gy[s_indPedestrian] = s_y;
}

void SimulationWhoRUwithAndWhereRUgoing::doStep(float s_dt)
{
    computeNeighbors();
    for(int i=0; i<g_nPedestrian; i++)
    {
        //setNearObstacles(i);
        update(i, s_dt);
    }
    for(int i=0; i<g_nPedestrian; i++)
    {
        agentlist[i]->pos.setX(getCenterxNext(i));
        agentlist[i]->pos.setY(getCenteryNext(i));
        agentlist[i]->vel.setX(getCenterVelocityxNext(i));
        agentlist[i]->vel.setY(getCenterVelocityyNext(i));
    }
}

void SimulationWhoRUwithAndWhereRUgoing::setPreferredVelocity(int s_indPedestrian, float s_x, float s_y)
{

}

float SimulationWhoRUwithAndWhereRUgoing::DCA(const QVector2D &deltaP, const QVector2D &deltaV)
{
    if(deltaV.lengthSquared() > 10E-6)
    {
        double ttca = - QVector2D::dotProduct(deltaP, deltaV) / deltaV.lengthSquared();
        return (deltaP + ttca * deltaV).length();
    }
    return 10E6;
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::socialForceYamaguchi(int index_i)
{
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::socialForceAlexandre(int index_i)
{
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::EnergyGradFixedObstacle(int index)
{
    Agent* agent = agentlist[index];

    QVector2D obs_rep_force(0,0);
    for(int j=0; j<obstacleList.size(); j++)
    {
        QPointF closest_pt_obs, closest_pt_agent, intersect_pt;
        distPt2Segment(agent->pos.toPointF(), obstacleList[j], &closest_pt_obs);
        closest_pt_agent = (agent->pos + (QVector2D(closest_pt_obs) - agent->pos) * agent->radius).toPointF();

        if(!LineCollision(closest_pt_agent, (closest_pt_agent + agent->vel.toPointF() *10), obstacleList[j].p1(), obstacleList[j].p2(), &intersect_pt))
            continue;

        QVector2D deltaP (closest_pt_obs - closest_pt_agent);
        float ttc = QVector2D(intersect_pt-closest_pt_agent).length() / (agent->vel.length()+epsilon);

        QVector2D grad_collision_cost_i = -deltaP.normalized() * expCost(deltaP.length() * ttc ,1);
        obs_rep_force += -grad_collision_cost_i;
    }
    return obs_rep_force;
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::EnergyGradDamping(int index)
{
    (void) index;
    return QVector2D(0,0);
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::EnergyGradSpeed(int index)
{
    float preferedSpeed = 1.;
    QVector2D &vel = agentlist[index]->vel;
    if(vel.length() > epsilon)
        return vel.normalized() * ( 2*(vel.length() - preferedSpeed) );
    return QVector2D(1,0) * preferedSpeed;
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::EnergyGradDirection(int index)
{
    Agent* a = agentlist[index];
    QVector2D gn = QVector2D(g_gx[index] - a->pos.x(), g_gy[index] - a->pos.y()).normalized();
    QVector2D vel = a->vel;
    if(vel.length() > epsilon)
    {
        // Gradient calculated by Matlab
        float diff_x = (gn.x()*vel.y()*vel.y() - gn.y()*vel.x()*vel.y())/pow(vel.lengthSquared(), 3/2.);
        float diff_y = (gn.y()*vel.x()*vel.x() - gn.x()*vel.y()*vel.x())/pow(vel.lengthSquared(), 3/2.);
        return QVector2D(-diff_x, -diff_y);
    }
    return QVector2D(1,0);
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::EnergyGradAttraction(int index)
{
    // TODO: first provide the index of groupmates
    return QVector2D(0,0);
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::EnergyGradGroup(int index)
{
    // TODO: first provide the index of groupmates
    return QVector2D(0,0);
}

QVector2D SimulationWhoRUwithAndWhereRUgoing::EnergyGradCollision(int index)
{
    Agent* agent_i = agentlist[index];

    QVector2D total_grad(0,0);
    for( int j=0; j<agent_i->neighbors.size(); j++)
    {
        Agent* agent_j = agent_i->neighbors[j];
        QVector2D deltaP = agent_j->pos - agent_i->pos;
        deltaP = deltaP.normalized() * fabs(deltaP.length() - agent_i->radius - agent_j->radius);
        QVector2D deltaV = agent_j->vel - agent_i->vel;

        double dca      = DCA(deltaP, deltaV);
        double dca_dx   = DCA(deltaP, deltaV - QVector2D(diff_h,0));
        double dca_dy   = DCA(deltaP, deltaV - QVector2D(0,diff_h));

        double dca_cost    = expCost(dca, sig_d);
        double dca_cost_dx = expCost(dca_dx, sig_d);
        double dca_cost_dy = expCost(dca_dy, sig_d);
        QVector2D grad_dca_cost_i ((dca_cost_dx-dca_cost)/diff_h, (dca_cost_dy-dca_cost)/diff_h);

        float dirs_dot_product = 0.5*(1 + QVector2D::dotProduct(deltaP.normalized(), agent_i->vel.normalized()));
        float fov_weight = pow(dirs_dot_product, beta_fov);
        float ttca_weight = exp(-deltaP.lengthSquared() /(2 * sig_w * sig_w));
        grad_dca_cost_i *= fov_weight * ttca_weight;
        total_grad += grad_dca_cost_i;
    }

    return total_grad;
}

void SimulationWhoRUwithAndWhereRUgoing::update(int ind, float dt)
{
    Agent* agent = agentlist[ind];

    QVector2D damping_force     = EnergyGradDamping(ind) * (-1);
    QVector2D speed_force       = EnergyGradSpeed(ind) * (-1);
    QVector2D direction_force   = EnergyGradDirection(ind) * (-1);
    QVector2D attraction_force  = EnergyGradAttraction(ind) * (-1);
    QVector2D group_force       = EnergyGradGroup(ind) * (-1);
    QVector2D collision_force   = EnergyGradCollision(ind) * (-1);
    QVector2D fix_obstacle_force= EnergyGradFixedObstacle(ind) * (-1);


    QVector2D total_force =
            lambda[0] * damping_force +
            lambda[1] * speed_force +
            lambda[2] * direction_force +
            lambda[3] * attraction_force +
            lambda[4] * group_force +
            lambda[5] * collision_force +
            fix_obstacle_force * 4;

    QVector2D next_vel = agent->vel + total_force * gama ;
    QVector2D next_pos = agent->pos + agent->vel * dt;

//    if(ind == 0)
//    {
//        float error_angle = atan2(error.y()+epsilon, error.x());
//        static int counter = 0;
//        dbuger()->getPlotter("angles")->addValue(d_angle, counter, "d_angle");
//        dbuger()->getPlotter("angles")->addValue(cur_angle, counter, "cur_angle");
//        dbuger()->getPlotter("angles")->addValue(error_angle, counter, "error_angle");
//        counter++;
//    }

    setNextState(ind, next_pos.x(), next_pos.y(), next_vel.x(), next_vel.y());
}

}
