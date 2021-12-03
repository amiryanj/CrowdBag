#include <boost/python.hpp>
#include <boost/python/list.hpp>
#include <boost/python/extract.hpp>
#include <string.h>
#include <iostream>
#include <stdio.h>

#include "craal-core/include/Simulation.h"
#include "models/include/SimulationBoids.h"
#include "models/include/SimulationHelbing.h"
#include "models/include/SimulationRVO2.h"
#include "models/include/SimulationPowerLaw.h"
using namespace craal;

namespace py = boost::python;

class CrowdSim
{
private:
    Simulation* sim;

public:
    CrowdSim(std::string model_name) {
        if(!model_name.compare("boids")) {
            sim = new SimulationBoids();
        }
        else if(!model_name.compare("helbing")) {
            sim = new SimulationHelbing();
        }
        else if(!model_name.compare("rvo2")) {
            sim = new SimulationRVO2();
        }
        else if(!model_name.compare("powerlaw")) {
            sim = new SimulationPowerLaw();
        }
        else
            std::cerr << "incorrect input" << std::endl;
    }

    inline py::list getObstacle(int s_obstacle) {
        py::list result;
        result.append(sim->g_obstaclesx[s_obstacle]);
        result.append(sim->g_obstaclesy[s_obstacle]);
        result.append(sim->g_obstacleex[s_obstacle]);
        result.append(sim->g_obstacleey[s_obstacle]);
        return result;
    }

    // ================================================
    inline float getObstacleStartx(int s_obstacle)
        {return sim->g_obstaclesx[s_obstacle];};
    /**
     * Get the Y coordinate of the starting vertex of an obstacle.
     * @param s_obstacle index of the obstacle
     */
    inline float getObstacleStarty(int s_obstacle)
        {return sim->g_obstaclesy[s_obstacle];};
    /**
     * Get the X coordinate of the ending vertex of an obstacle.
     * @param s_obstacle index of the obstacle
     */
    inline float getObstacleEndx(int s_obstacle)
        {return sim->g_obstacleex[s_obstacle];};
    /**
     * Get the Y coordinate of the ending vertex of an obstacle.
     * @param s_obstacle index of the obstacle
     */
    inline float getObstacleEndy(int s_obstacle)
        {return sim->g_obstacleey[s_obstacle];};


    inline py::list getCenter(int s_indPedestrian) {
        py::list result;
        result.append(sim->g_centerx[s_indPedestrian]);
        result.append(sim->g_centery[s_indPedestrian]);
        return result;
    }
    /**
     * Get the X position coordinate of an agent at the frame set by
     * @ref Simulation::setTime().
     *@param s_indPedestrian index of the agent
     */
    inline float getCenterx(int s_indPedestrian)
        {return sim->g_centerx[s_indPedestrian][((int)(sim->g_time*(sim->g_nMeasure-1)))];};
    /**
     * Get the Y position coordinate of an agent at the frame set by
     * @ref Simulation::setTime().
     *@param s_indPedestrian index of the agent
     */
    inline float getCentery(int s_indPedestrian)
        {return sim->g_centery[s_indPedestrian][((int)(sim->g_time*(sim->g_nMeasure-1)))];};


    inline py::list getCenterVelocity(int s_indPedestrian) {
        py::list result;
        result.append(sim->g_centerVelocityx[s_indPedestrian]);
        result.append(sim->g_centerVelocityy[s_indPedestrian]);
        return result;
    }
    /**
     * Get the X velocity coordinate of an agent at the frame set by
     * @ref Simulation::setTime().
     *@param s_indPedestrian index of the agent
     */
    inline float getCenterVelocityx(int s_indPedestrian)
        {return sim->g_centerVelocityx[s_indPedestrian][((int)(sim->g_time*(sim->g_nMeasure-1)))];};
    /**
     * Get the Y velocity coordinate of an agent at the frame set by
     * @ref Simulation::setTime().
     *@param s_indPedestrian index of the agent
     */
    inline float getCenterVelocityy(int s_indPedestrian)
        {return sim->g_centerVelocityy[s_indPedestrian][((int)(sim->g_time*(sim->g_nMeasure-1)))];};


    inline py::list getCenterNext(int s_indPedestrian) {
        py::list result;
        result.append(sim->g_centerxNext[s_indPedestrian]);
        result.append(sim->g_centeryNext[s_indPedestrian]);
        return result;
    }
    /**
     * Get the next X position coordinate of an agent.
     *@param s_indPedestrian index of the agent
     */
    inline float getCenterxNext(int s_indPedestrian)
        {return sim->g_centerxNext[s_indPedestrian];};
    /**
     * Get the next Y position coordinate of an agent.
     *@param s_indPedestrian index of the agent
     */
    inline float getCenteryNext(int s_indPedestrian)
        {return sim->g_centeryNext[s_indPedestrian];};


    inline py::list getCenterVelocityNext(int s_indPedestrian) {
        py::list result;
        result.append(sim->g_centerVelocityxNext[s_indPedestrian]);
        result.append(sim->g_centerVelocityyNext[s_indPedestrian]);
        return result;
    }
    /**
     * Get the next X velocity coordinate of an agent.
     *@param s_indPedestrian index of the agent
     */
    inline float getCenterVelocityxNext(int s_indPedestrian)
        {return sim->g_centerVelocityxNext[s_indPedestrian];};
    /**
     * Get the next Y velocity coordinate of an agent.
     *@param s_indPedestrian index of the agent
     */
    inline float getCenterVelocityyNext(int s_indPedestrian)
        {return sim->g_centerVelocityyNext[s_indPedestrian];};



    /**
     * Get the X position coordinate of an agent at a given frame.
     * @param s_indPedestrian index of the agent
     * @param s_indMeasure index of the frame
     */
//    inline float getCenterx(int s_indPedestrian, int s_indMeasure)
//        {return sim->g_centerx[s_indPedestrian][s_indMeasure];};
//    /**
//     * Get the Y position coordinate of an agent at a given frame.
//     * @param s_indPedestrian index of the agent
//     * @param s_indMeasure index of the frame
//     */
//    inline float getCentery(int s_indPedestrian, int s_indMeasure)
//        {return sim->g_centery[s_indPedestrian][s_indMeasure];};
//
//    /**
//     * Get the X velocity coordinate of an agent at a given frame.
//     * @param s_indPedestrian index of the agent
//     * @param s_indMeasure index of the frame
//     */
//    inline float getCenterVelocityx(int s_indPedestrian, int s_indMeasure)
//        {return sim->g_centerVelocityx[s_indPedestrian][s_indMeasure];};
//    /**
//     * Get the Y velocity coordinate of an agent at a given frame.
//     * @param s_indPedestrian index of the agent
//     * @param s_indMeasure index of the frame
//     */
//    inline float getCenterVelocityy(int s_indPedestrian, int s_indMeasure)
//        {return sim->g_centerVelocityy[s_indPedestrian][s_indMeasure];}

    // ================ Virtual Functions =============
//    void init() {
//        sim->init();
//    }

    int getNPedestrian() {
        return sim->getNPedestrian();
    }
    int getNObstacle()   {
        return sim->getNObstacle();
    }

    void reset() {
        sim->reset();
    }

    void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy) {
        sim->addObstacleCoords(s_startx, s_starty, s_endx, s_endy);
    }

    void setPosition(int s_indPedestrian, float s_x, float s_y) {
        sim->setPosition(s_indPedestrian, s_x, s_y);
    }
    void setVelocity(int s_indPedestrian, float s_x, float s_y) {
        sim->setVelocity(s_indPedestrian, s_x, s_y);
    }
    void setGoal(int s_indPedestrian, float s_x, float s_y) {
        sim->setGoal(s_indPedestrian, s_x, s_y);
    }

    inline void setTime(float s_time = 0) {
        sim->g_time = s_time;
    }

    void doStep(float s_dt = 0.01f) {
        sim->doStep(s_dt);
    }

    // =====================
    void init(int s_nPedestrian) {
        sim->initSimulation(s_nPedestrian);
    }    

    // =====================
    void setAgentRadius(int s_indPedestrian, float r) {
        SimulationBoids* boids = dynamic_cast<SimulationBoids*>(sim);
        SimulationHelbing* helbing = dynamic_cast<SimulationHelbing*>(sim);
        SimulationPowerLaw* powerlaw = dynamic_cast<SimulationPowerLaw*>(sim);
        SimulationRVO2* rvo = dynamic_cast<SimulationRVO2*>(sim);

        if(boids) boids->setAgentRadius(s_indPedestrian, r);
        else if (helbing) helbing->setAgentRadius(s_indPedestrian, r);
        else if (powerlaw) powerlaw->setAgentRadius(s_indPedestrian, r);
        else if (rvo) rvo->setAgentRadius(s_indPedestrian, r);
        else std::cerr << "sim is not initiated correctly" << std::endl;
    }

    void setAgentSpeed(int s_indPedestrian, float s) {
        SimulationBoids* boids = dynamic_cast<SimulationBoids*>(sim);
        SimulationHelbing* helbing = dynamic_cast<SimulationHelbing*>(sim);
        SimulationPowerLaw* powerlaw = dynamic_cast<SimulationPowerLaw*>(sim);
        SimulationRVO2* rvo = dynamic_cast<SimulationRVO2*>(sim);

        if(boids) boids->setAgentSpeed(s_indPedestrian, s);
        else if (helbing) helbing->setAgentSpeed(s_indPedestrian, s);
        else if (powerlaw) powerlaw->setAgentPrefSpeed(s_indPedestrian, s);
        else if (rvo) rvo->setAgentMaxSpeed(s_indPedestrian, s);
        else std::cerr << "sim is not initiated correctly" << std::endl;
    }

    void setAgentNeighborDist(int s_indPedestrian, float d) {
        SimulationPowerLaw* powerlaw = dynamic_cast<SimulationPowerLaw*>(sim);
        SimulationRVO2* rvo = dynamic_cast<SimulationRVO2*>(sim);

        // for Helbing and Boids it is fixed = 5.4 * agent.radius
        if (powerlaw) powerlaw->setAgentNeighborDist(s_indPedestrian, d);
        else if (rvo) rvo->setAgentNeighborDist(s_indPedestrian, d);
        else std::cerr << "irrelevant parameter or is not initiated correctly" << std::endl;
    }

    void setAgentTimeHorizon(int s_indPedestrian, float t) {
        SimulationPowerLaw* powerlaw = dynamic_cast<SimulationPowerLaw*>(sim);
        SimulationRVO2* rvo = dynamic_cast<SimulationRVO2*>(sim);

        // for Helbing and Boids it is fixed = 5.4 * agent.radius
        if (powerlaw) powerlaw->setAgentT0(s_indPedestrian, t);
        else if (rvo) {
            rvo->setAgentTimeHorizon(s_indPedestrian, t);
            rvo->setAgentTimeHorizonObst(s_indPedestrian, t);
        }
        else std::cerr << "irrelevant parameter or is not initiated correctly" << std::endl;
    }

    // =====================
    float getPi() { // test function
        return 3.16f;
    }

};


