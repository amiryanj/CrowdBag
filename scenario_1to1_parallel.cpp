#include "crowdbag.hpp"
#include <iostream>

using namespace std;

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

    int n_peds = 2;

    sim.init(n_peds);
    sim.setTime(0);

    // initial positions
	sim.setAgent(0, -ratio, 0, ratio, 0, radius, speed);
	sim.setAgent(1, ratio, 0, -ratio, 0, radius, speed);

    // Simulation
    float dt = 0.05;
    float total_time = 10; //seconds

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
