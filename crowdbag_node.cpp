#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <rosgraph_msgs/Clock.h>
#include "./third-party/tinyxml2.h"
#include "./crowdbag.hpp"

using namespace tinyxml2;
using namespace std;

struct Vec3 {
    float x, y, z;
};

struct Line {
    float ps_x, ps_y, pe_x, pe_y;
};

void loadXML(const char *filename, CrowdSim& sim)
{
    int n_peds = 0;

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
        return;
    }

    sim.init(n_peds);
    for (int i =0; i<n_peds; i++) {
        sim.setPosition(i, positions[i].x, positions[i].y);
        sim.setGoal(i, goals[i].x, goals[i].y);
        sim.setAgentRadius(i, params_radius[i]);
        sim.setAgentSpeed(i, params_max_speed[i]);
        sim.setAgentTimeHorizon(i, params_time_horizon[i]);
        sim.setAgentNeighborDist(i, params_neighbor_dist[i]);
    }
}

class Listener {
    CrowdSim* sim;
    ros::NodeHandle* node;
    ros::Publisher * pub;
public:
    Listener(ros::NodeHandle* node_, CrowdSim* sim_) : node(node_), sim(sim_) {
        *pub = node->advertise<std_msgs::String>("crowd" //
                                                     , 100 /* queue capacity */ );
    }
    void clock_callback(const rosgraph_msgs::Clock& msg)
    {
        float dt = 0.1;
        /// @todo
        // dt = msg. ...

        this->sim->doStep(dt);
        int N = sim->getNPedestrian();

        // CrowdMsg crowd_msg;
        for (int i=0; i<N; i++) {
            float px_i = sim->getCenterxNext(i);
            float py_i = sim->getCenteryNext(i);
            float vx_i = sim->getCenterVelocityxNext(i);
            float vy_i = sim->getCenterVelocityxNext(i);
            // crowd_msg.add(...)
            // pub.publish(crowd_msg);
        }
    }
};



/// Standard C++ entry point
int main(int argc, char** argv)
{
    /// Announce this program to the ROS master as a "node" called "hello_world_node"
    ros::init(argc, argv, "crowdbag_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);

    CrowdSim sim(std::string("rvo2"));  ///@todo: set from argv

    loadXML("/home/cyrus/workspace2/CrowdBotUnity/Scenario/CrowdBot/FollowBot/GeneratedScenario.xml", sim); ///@todo: set from argv
    sim.setTime(0);

    Listener l(&node, &sim);

//    int msg_counter = 0;
    while(ros::ok())
    {
//        std_msgs::String msg;
//        std::stringstream ss;
//        ss << "hello world " << msg_counter++;
//        msg.data = ss.str();
//        pub.publish(msg);
//        /// Process ROS callbacks until receiving a SIGINT (ctrl-c)
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Stop the node's resources
    ros::shutdown();
    // Exit tranquilly
    return 0;
}
