#include <iostream>
#include "./third-party/tinyxml2.h"
#include "./crowdbag.hpp"

using namespace std;
using namespace tinyxml2;

struct Vec3 {
    float x, y, z;
};

struct Line {
    float ps_x, ps_y, pe_x, pe_y;
};

class XMLLoader
{
public:
    XMLLoader() {}
    CrowdSim* load(const char *filename)
    {
        cout << "1" << endl;

        int n_peds = 0;
        string model_name = "rvo2";  /// @todo: read from xml
        std::vector<Line> obstacles;
        std::vector<Vec3> positions, rotations, goals;
        std::vector<float> params_radius, params_max_speed, params_neighbor_dist, params_time_horizon;

        try {
            XMLDocument doc;

            XMLError eResult = doc.LoadFile(filename);
            if(eResult != 0) throw std::invalid_argument( "xml file is invalid!");
            XMLHandle docHandle(&doc);

            XMLElement *root = doc.RootElement();
            if(root == nullptr) throw std::invalid_argument( "no root found in xml file!");


            // read the agents + goals
            XMLElement *element_agents = root->FirstChildElement("agents");
            if(element_agents == nullptr) throw std::invalid_argument( "There is no element \"agets\" in xml file");

            XMLElement * element_agent = element_agents->FirstChildElement("agent");
            while (element_agent!= nullptr) {
                n_peds ++ ;
                Vec3 pos, rot, goal;
                XMLElement * element_pos = element_agent->FirstChildElement("TrialRegularAgent")->FirstChildElement("Position");
                pos.x = element_pos->FloatAttribute("x");
                pos.y = element_pos->FloatAttribute("y");
                pos.z = element_pos->FloatAttribute("z");
                positions.push_back(pos);

                XMLElement * element_rot = element_agent->FirstChildElement("TrialRegularAgent")->FirstChildElement("Rotation");
                rot.x = element_rot->FloatAttribute("x");
                rot.y = element_rot->FloatAttribute("y");
                rot.z = element_rot->FloatAttribute("z");
                rotations.push_back(rot);

                XMLElement * element_goal = element_agent->FirstChildElement("TrialRegularAgent")->FirstChildElement("controlLaw")
                        ->FirstChildElement("LawGoals")->FirstChildElement("Goals")->FirstChildElement("Goal");
                goal.x = element_goal->FloatAttribute("x");
                goal.y = element_goal->FloatAttribute("y");
                goal.z = element_goal->FloatAttribute("z");
                goals.push_back(goal);

                XMLElement * element_params = element_agent->FirstChildElement("TrialRegularAgent")->FirstChildElement("controlSim")
                        ->FirstChildElement("RVOconfig");
                params_radius.push_back(element_params->FloatAttribute("radius"));
                params_max_speed.push_back(element_params->FloatAttribute("maxSpeed"));
                params_time_horizon.push_back(element_params->FloatAttribute("timeHorizon"));
                params_neighbor_dist.push_back(element_params->FloatAttribute("neighborDist"));

                element_agent = element_agent->NextSiblingElement("agent");
            }
            if(n_peds == 0) throw std::invalid_argument( "Couldn't find any agent in xml file");

            /// @todo: read the obstacles
            XMLElement* element_obstacles = NULL;

        } catch (std::exception& e) {
            std::cerr << "exception: " << e.what() << std::endl;
            return NULL;
        }
        cout << "2" << endl;

        CrowdSim* sim = new CrowdSim(std::string(model_name));
        sim->init(n_peds);
        for (int i =0; i<n_peds; i++) {
            sim->setPosition(i, positions[i].x, positions[i].y);
            sim->setGoal(i, goals[i].x, goals[i].y);
            sim->setAgentRadius(i, params_radius[i]);
            sim->setAgentSpeed(i, params_max_speed[i]);
            sim->setAgentTimeHorizon(i, params_time_horizon[i]);
            sim->setAgentNeighborDist(i, params_neighbor_dist[i]);
        }
        cout << "3" << endl;
        return sim;
    }


};

