#include "crowdbag.hpp"
#include <iostream>
#include "./third-party/tinyxml2.h"

using namespace tinyxml2;
using namespace std;

struct Vec3 {
    float x, y, z;
};

struct Line {
    float ps_x, ps_y, pe_x, pe_y;
};

int main(int argc, char *argv[])
{
    float radius = 0.3;
    float speed = 1.3;
    float time_horizon = 2;
    float neighbor_dist = 5;

    float ratio = 5; // use this to scale up the initial distance to collision

    string simType= "rvo2";
    if(argc > 1) simType = argv[1];

    CrowdSim sim(simType);
    // CrowdSim sim("rvo2");
    // CrowdSim sim("helbing");
    // CrowdSim sim("vision");
    // CrowdSim sim("boids");

    int n_peds = 10;

	float xPos[10] = { 7.7880f,    0.9082f,    4.4009f,    4.5742f,    6.3771f,    2.4071f,    6.9514f,    2.5479f,    3.4446f,    6.0217f};
	float yPos[10] = { -0.2296f,   -0.7006f,    0.0814f,    1.1261f,    1.3731f,    0.5284f,   -1.2960f,   -0.8279f,    0.8416f,   -0.3397f};

    // the additional guy
    n_peds++;

    // init the simulation
    sim.init(n_peds);
    for (int i =0; i<n_peds-1; i++) {
		sim.setAgent(i, xPos[i], yPos[i], xPos[i]-50000, yPos[i], radius, speed);
    }

    // set the 1 guy left
	sim.setAgent(n_peds-1, -ratio, 0, 50000, 0, radius, speed);

    sim.setTime(0);

    // Simulation
    float dt = 0.05;
    float total_time = 20; //seconds

    for(int k=0; k < total_time/dt; k++) {

        float t = dt * k;

        sim.doStep(dt);

        printf("%.3f", t);

        for(int i=0; i<n_peds; i++) {
            float px_i = sim.getCenterxNext(i);
            float py_i = sim.getCenteryNext(i);
            printf(",%.3f,%.3f", px_i, py_i);
        }
        std::cout << "\r";


    }

}
