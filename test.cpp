#include "crowdbag.hpp"
#include <iostream>

int main()
{
    CrowdSim sim("boids");

    int N = 10;
    sim.init(N);
    sim.setTime(0);

    for (int i =0; i<N; i++) {
        sim.setPosition(i, 0, i);
        sim.setGoal(i, 10, i);
        sim.setAgentSpeed(i, 2);
    }

    float dt = 0.01;
    for(int k=0; k<100; k++) {
        float t = dt * k;
        sim.doStep(dt);
        for(int i=0; i<N; i++) {
            float px_i = sim.getCenterxNext(i);
            float py_i = sim.getCenteryNext(i);
            float vx_i = sim.getCenterVelocityxNext(i);
            float vy_i = sim.getCenterVelocityxNext(i);
            printf("%d, %d, %.3f, %.3f", int(round(t*100)), i, px_i, py_i);
            std::cout << std::endl;
        }


    }

}
