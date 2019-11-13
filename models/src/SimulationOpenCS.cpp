//========================================================================
/*!
  @file		SimulationOpenCS.cpp
  @class	SimulationOpenCS
  @date    	08/6/2018
  @brief
  @author  Javad Amirian, (C) 2018
*/
//========================================================================


#include "../include/SimulationOpenCS.h"
#include "geometrymath.h"
#include "DEFINITIONS.h"

#include "debug-tools/debugfactory.h"
using namespace sdbug;


namespace craal {

SimulationOpenCS::SimulationOpenCS()
{
    addParam("Speed", 1.2, PDF(1, 2, PDF::NORMAL, 1.2, 0.5));
    addParam("sig_d", 0.4, PDF(0.5, 4, PDF::NORMAL, 1.5, 1.0));
    addParam("sig_w", 2.5, PDF(0.2, 4, PDF::NORMAL, 1.5, 1.0));

    name = "OpenCS";
    randomVect = QVector2D(PDF(-randRange, randRange).get(), PDF(-randRange, randRange).get());
}

SimulationOpenCS::~SimulationOpenCS()
{
    p_delete2();
}

void SimulationOpenCS::p_delete2()
{
    // TODO: complete it
    if (g_gx) delete [] g_gx;
    if (g_gy) delete [] g_gy;
    g_gx = 0;
    g_gy = 0;
    for(int i=0; i<agentlist.size(); i++)
        delete agentlist[i];
    agentlist.clear();
    obstacleList.clear();
}

void SimulationOpenCS::computeNeighbors()
{
    for(int i=0; i<agentlist.size(); i++)
        agentlist[i]->neighbors.clear();

    static float neighborhood_thre_sqr = pow(DEF_RADIUS * 10, 2);

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

void SimulationOpenCS::setNearObstacles(int index)
{
    Agent* a = agentlist[index];
    double dist, minDist2, minDist = 2*a->radius*a->radius;
    minDist2 = minDist;
    for (int j = 0; j < obstacleList.size(); j++){
        //closestPointLine(a.pos,obstacles[j].a,obstacles[j].b,dist);
        dist = DistancePointLineSqr(a->pos,
                                    QVector2D(obstacleList[j].p1()),
                                    QVector2D(obstacleList[j].p2()));
        if (dist < minDist2){
            if (dist < minDist){
                minDist2 = minDist;
                a->nearObstacle[1] = a->nearObstacle[0];
                minDist = dist;
                a->nearObstacle[0] = obstacleList[j];
            }else{
                minDist2 = dist;
                a->nearObstacle[1] = obstacleList[j];
            }
        }
    }
}

void SimulationOpenCS::init()
{
    p_delete2();

    g_gx = new float[g_nPedestrian];
    g_gy = new float[g_nPedestrian];

    agentlist.resize(g_nPedestrian);
    for(int i=0; i<g_nPedestrian; i++)
    {
        agentlist[i] = new Agent();
        agentlist[i]->radius = DEF_RADIUS;

        g_params[i*g_nbParams+0] = g_pdfs[0].get();
        g_params[i*g_nbParams+1] = g_pdfs[1].get();
        g_params[i*g_nbParams+2] = g_pdfs[2].get();
    }
}

void SimulationOpenCS::reset()
{
    //TO DO: copy g_paras to lambdas
}

void SimulationOpenCS::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
    QLineF obs;
    obs.setP1(QPointF(s_startx, s_starty));
    obs.setP2(QPointF(s_endx, s_endy));
    obstacleList.append(obs);
}

void SimulationOpenCS::setPosition(int s_indPedestrian, float s_x, float s_y)
{
    agentlist[s_indPedestrian]->pos.setX(s_x);
    agentlist[s_indPedestrian]->pos.setY(s_y);
}

void SimulationOpenCS::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
    agentlist[s_indPedestrian]->vel.setX(s_x);
    agentlist[s_indPedestrian]->vel.setY(s_y);
}

void SimulationOpenCS::setGoal(int s_indPedestrian, float s_x, float s_y)
{
    g_gx[s_indPedestrian] = s_x;
    g_gy[s_indPedestrian] = s_y;
}

void SimulationOpenCS::doStep(float s_dt)
{
    computeNeighbors();
    for(int i=0; i<g_nPedestrian; i++)
    {
        //setNearObstacles(i);
        FixedUpdate(i, s_dt);
    }
    for(int i=0; i<g_nPedestrian; i++)
    {
        agentlist[i]->pos.setX(getCenterxNext(i));
        agentlist[i]->pos.setY(getCenteryNext(i));
        agentlist[i]->vel.setX(getCenterVelocityxNext(i));
        agentlist[i]->vel.setY(getCenterVelocityyNext(i));
    }
}

float SimulationOpenCS::TTCA(const QVector2D &deltaP, const QVector2D &deltaV)
{
    if(deltaV.lengthSquared() > 10E-6)
        return - QVector2D::dotProduct(deltaP, deltaV) / deltaV.lengthSquared();
    return 10E6;
}

float SimulationOpenCS::DCA(const QVector2D &deltaP, const QVector2D &deltaV)
{
    return (deltaP + TTCA(deltaP, deltaV) * deltaV).length();
}

QVector2D SimulationOpenCS::TTCA_Grad(const QVector2D &deltaP, const QVector2D &deltaV)
{
    QVector2D dv_x (epsilon, 0.);
    QVector2D dv_y (0., epsilon);
    float gradTTCAi_x = 0.5 * ( TTCA(deltaP, deltaV + dv_x) - TTCA(deltaP, deltaV - dv_x) );
    float gradTTCAi_y = 0.5 * ( TTCA(deltaP, deltaV + dv_y) - TTCA(deltaP, deltaV - dv_y) );
    return QVector2D(gradTTCAi_x/epsilon, gradTTCAi_y/epsilon);
}

QVector2D SimulationOpenCS::DCA_Grad(const QVector2D &deltaP, const QVector2D &deltaV)
{
    QVector2D dv_x (epsilon, 0.);
    QVector2D dv_y (0., epsilon);
    float gradDCAi_x = 0.5 * ( DCA (deltaP, deltaV + dv_x) - DCA (deltaP, deltaV - dv_x) );
    float gradDCAi_y = 0.5 * ( DCA (deltaP, deltaV + dv_y) - DCA (deltaP, deltaV - dv_y) );
    return QVector2D (gradDCAi_x/epsilon , gradDCAi_y/epsilon);
}

QVector2D SimulationOpenCS::socialForceFabien(int index_i)
{
    QVector2D GOC (0, 0);

    Agent* a = agentlist[index_i];
    int N = 0;
    for( int j=0; j<g_nPedestrian; j++)
    {
        if(j == index_i)
            continue;

        Agent* agent_j = agentlist[j];
        QVector2D deltaP = agent_j->pos - a->pos;
        QVector2D deltaV = agent_j->vel - a->vel;

        float ttca = TTCA(deltaP, deltaV);
        //if(ttca < 0)
        // continue;
        float dca  = DCA (deltaP, deltaV);
        float OCi  = exp( -0.5 * ( pow(ttca / SigTTCA, 2) + pow(dca / SigDCA, 2) ) );

        QVector2D dv_x (epsilon, 0.);
        QVector2D dv_y (0., epsilon);
        float gradTTCAi_x = 0.5 * ( TTCA(deltaP, deltaV + dv_x) - TTCA(deltaP, deltaV - dv_x) );
        float gradTTCAi_y = 0.5 * ( TTCA(deltaP, deltaV + dv_y) - TTCA(deltaP, deltaV - dv_y) );
        float gradDCAi_x  = 0.5 * ( DCA (deltaP, deltaV + dv_x) - DCA (deltaP, deltaV - dv_x) );
        float gradDCAi_y  = 0.5 * ( DCA (deltaP, deltaV + dv_y) - DCA (deltaP, deltaV - dv_y) );

        QVector2D gradTTCAi (gradTTCAi_x, gradTTCAi_y);
        QVector2D gradDCAi  (gradDCAi_x , gradDCAi_y);

        gradTTCAi.normalize();
        gradDCAi.normalize();
        QVector2D obs_grad_i = OCi * (gradTTCAi * ttca / SigTTCA + gradDCAi * dca / SigDCA);

        float DistanceCost = exp( -0.5 * pow( deltaP.length() / SigPS, 2) ) ;

        float weightFov = 1 - QVector2D::dotProduct(deltaP.normalized(), agent_j->vel.normalized());
        if(weightFov >= 1.f)
            weightFov = 1.f;

        GOC += weightFov * obs_grad_i * DistanceCost;
        N++;
    }

    if( N == 0 || std::isnan(GOC.x()) || std::isnan(GOC.y()))
        return QVector2D(0,0);
    else
        return GOC * (1. / N);
}

QVector2D SimulationOpenCS::socialForceAlexandre(int index_i)
{
    Agent* agent_i = agentlist[index_i];
    float &sig_d = g_params[index_i*g_nbParams+2];
    float &sig_w = g_params[index_i*g_nbParams+2];

    QVector2D total_col_force (0,0);
    //int N = 0;
    for( int j=0; j<agent_i->neighbors.size(); j++)
    {
        Agent* agent_j = agent_i->neighbors[j];
        QVector2D deltaP = agent_j->pos - agent_i->pos;
        deltaP = deltaP.normalized() * max(deltaP.length() - agent_i->radius - agent_j->radius, 0.f);
        QVector2D deltaV = agent_j->vel - agent_i->vel;

        double dca_cost    = expCost(DCA(deltaP, deltaV),sig_d);
        double dca_cost_dx = expCost(DCA(deltaP, deltaV - QVector2D(epsilon,0)), sig_d);
        double dca_cost_dy = expCost(DCA(deltaP, deltaV - QVector2D(0,epsilon)) ,sig_d);
        QVector2D grad_dca_cost_i ((dca_cost_dx-dca_cost)/epsilon, (dca_cost_dy-dca_cost)/epsilon);

        float dirs_dot_product = 0.5*(1 + QVector2D::dotProduct(deltaP.normalized(), agent_i->vel.normalized()));
        float fov_weight = pow(dirs_dot_product, beta_fov);
        float ttca_weight = exp(-0.5 * deltaP.length() / sig_w);
        float same_vel_weight = exp(-pow(TTCA(deltaP, deltaV), 2.0));
        grad_dca_cost_i *= fov_weight * ttca_weight * same_vel_weight;

        /*
        QVector2D rep_force = deltaP.normalized();
        rep_force *= fov_weight * ttca_weight * same_vel_weight; */

        total_col_force += -grad_dca_cost_i /*+ rep_force*/;
    }

    return total_col_force;
}

QVector2D SimulationOpenCS::goalForceFabien(int index)
{
    Agent* a = agentlist[index];
    QVector2D &currentSpeed = a->vel;
    QVector2D goalDir(g_gx[index] - a->pos.x(), g_gy[index] - a->pos.y());
    float &preferedSpeed = g_params[index* g_nbParams + 0];
    QVector2D deltaSpeed = currentSpeed - goalDir.normalized() * preferedSpeed;

    return deltaSpeed/gaussianCost(QVector2D::dotProduct(deltaSpeed,deltaSpeed), 1);
}

QVector2D SimulationOpenCS::goalForceYamaguchi(int index)
{
    float preferedSpeed = 1.;
    Agent* a = agentlist[index];
    QVector2D &vel = a->vel;
    if(vel.length() < epsilon)
    {
        return QVector2D(1,0) * preferedSpeed;
    }

    QVector2D speed_alignment_force = vel.normalized() * ( 2*(vel.length() - preferedSpeed) );

    QVector2D gn = QVector2D(g_gx[index] - a->pos.x(), g_gy[index] - a->pos.y()).normalized();
    // Gradient calculated by Matlab
    float diff_x = (gn.x()*vel.y()*vel.y() - gn.y()*vel.x()*vel.y())/pow(vel.lengthSquared(), 3/2.);
    float diff_y = (gn.y()*vel.x()*vel.x() - gn.x()*vel.y()*vel.x())/pow(vel.lengthSquared(), 3/2.);
    QVector2D direction_alignment_force(-diff_x, -diff_y);

    return speed_alignment_force + direction_alignment_force;
}

QVector2D SimulationOpenCS::fixedObstacleForce(int index)
{
    Agent* agent = agentlist[index];
    QVector2D obs_rep_force(0,0);
    float &sig_d = g_params[index*g_nbParams+2];
    float &sig_w = g_params[index*g_nbParams+2];

    for(int j=0; j<obstacleList.size(); j++)
    {
        QPointF closest_pt_obs, closest_pt_agent, intersect_pt;
        distPt2Segment(agent->pos.toPointF(), obstacleList[j], &closest_pt_obs);
        closest_pt_agent = (agent->pos + (QVector2D(closest_pt_obs) - agent->pos) * agent->radius).toPointF();

        if(!LineCollision(closest_pt_agent, (closest_pt_agent + agent->vel.toPointF() *10), obstacleList[j].p1(), obstacleList[j].p2(), &intersect_pt))
            continue;

        QVector2D deltaP (closest_pt_obs - closest_pt_agent);
        float ttc = QVector2D(intersect_pt-closest_pt_agent).length() / (agent->vel.length()+epsilon);

        QVector2D grad_collision_cost_i = -deltaP.normalized() * expCost(deltaP.length() * ttc , 0.3);
        obs_rep_force += grad_collision_cost_i;
    }
    return obs_rep_force;
}

void SimulationOpenCS::FixedUpdate(int ind, float dt)
{    
    Agent* agent = agentlist[ind];
    float LambdaMov = 0.25;
    QVector2D goal_force = -goalForceYamaguchi(ind) * LambdaMov;
    QVector2D social_force = socialForceAlexandre(ind) * 3.0;
    QVector2D obs_force = fixedObstacleForce(ind) * 1.2;

    agent->tmp_dca_vec = social_force;
    agent->tmp_goal_vec = goal_force;
    agent->tmp_rep_vec = obs_force;

    QVector2D error = goal_force + social_force + obs_force + randomVect;
    QVector2D next_vel = agent->vel + error ;

    float cur_angle = atan2(agent->vel.y()+10E-9, agent->vel.x());
    float next_vel_angle = atan2(next_vel.y(), next_vel.x());
    float d_speed = next_vel.length() - agent->vel.length();
    float d_angle = next_vel_angle - cur_angle;
    while(fabs(d_angle) > M_PI) {
        if(d_angle > M_PI) d_angle -= 2*M_PI;
        else d_angle += 2*M_PI;
    }
    float new_speed = agent->vel.length() + d_speed * 0.5;
    float new_angle = atan2(agent->vel.y(), agent->vel.x()) + sigmoid(d_angle, 0.5);
    next_vel = QVector2D(new_speed* cos(new_angle), new_speed* sin(new_angle));
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
