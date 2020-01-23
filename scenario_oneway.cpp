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
    string file = "";

    if(argc > 1) simType = argv[1];
    if(argc > 2) file = argv[2];

    CrowdSim sim(simType);
    // CrowdSim sim("rvo2");
    // CrowdSim sim("helbing");
    // CrowdSim sim("vision");
    // CrowdSim sim("boids");


    int n_peds = 0;

    std::vector<Vec3> positions;
    std::vector<float> params_pref_speed;

    try {
        XMLDocument doc;

        const char* filename = file.c_str();
        XMLError eResult = doc.LoadFile(filename);
        if(eResult != 0) throw std::invalid_argument( "xml file is invalid!");
        XMLHandle docHandle(&doc);

        XMLElement *root = doc.RootElement();
        if(root == nullptr) throw std::invalid_argument( "no root found in xml file!");

        // read the agents + goals
        XMLElement *element_agents = root->FirstChildElement("agents");
        if(element_agents == nullptr) throw std::invalid_argument( "There is no element \"agents\" in xml file");

        XMLElement * element_agent = element_agents->FirstChildElement("agent");
        while (element_agent!= nullptr) {
            n_peds ++ ;
            Vec3 pos;
            XMLElement * element_pos = element_agent->FirstChildElement("TrialRegularAgent")->FirstChildElement("Position");
            pos.x = element_pos->FloatAttribute("x");
            pos.y = element_pos->FloatAttribute("y");
            pos.z = element_pos->FloatAttribute("z");
            positions.push_back(pos);

            XMLElement * element_params = element_agent->FirstChildElement("TrialRegularAgent")->FirstChildElement("controlLaw")
                    ->FirstChildElement("LawWaypoints");
            params_pref_speed.push_back(element_params->FloatAttribute("speedDefault"));

            element_agent = element_agent->NextSiblingElement("agent");
        }
        if(n_peds == 0) throw std::invalid_argument( "Couldn't find any agent in xml file");

    } catch (std::exception& e) {
        std::cerr << "exception: " << e.what() << std::endl;
        return 0;
    }

    // init the simulation
    sim.init(n_peds);
    for (int i =0; i<n_peds; i++) {
		sim.setAgent(i, positions[i].x, positions[i].y, positions[i].x + 5000, positions[i].y, radius, params_pref_speed[i]);
    }

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
