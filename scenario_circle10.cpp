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

    int n_peds = 10;

    sim.init(n_peds);
    sim.setTime(0);
	
	float xPos[10] = { 5.0000f,    4.0425f,    1.5368f,   -1.5575f,   -4.0553f,   -5.0000f,   -4.0296f,   -1.5160f,    1.5783f,    4.0681f};
	float yPos[10] = { 0.0000f,    2.9425f,    4.7580f,    4.7512f,    2.9248f,   -0.0218f,   -2.9601f,   -4.7646f,   -4.7444f,   -2.9070f};
         

    // initial positions
    for (int i =0; i<n_peds; i++) {
        sim.setAgent(i, xPos[i], yPos[i], -xPos[i], -yPos[i], radius, speed);
    }

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
