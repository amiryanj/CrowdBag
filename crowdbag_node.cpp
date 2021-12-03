#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "rosgraph_msgs/Clock.h"


#include "./xml_loader.hpp"
//#include "./crowdbag.hpp"
using namespace std;


class Listener {
    CrowdSim* sim;
    ros::NodeHandle* node;
    ros::Publisher pub;

public:
    Listener(ros::NodeHandle* node_, CrowdSim* sim_) : node(node_), sim(sim_) {
        pub = this->node->advertise<std_msgs::String>("crowd", 100);
        cout << "4" << endl;
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
    if(argc < 2) {
        cerr << "Error! please set the xml file as input!" << endl;
        exit(0);
    }

    ros::init(argc, argv, "crowdbag_node");
    ros::NodeHandle node;
    ros::Rate loop_rate(10);

    CrowdSim *sim = XMLLoader().load(argv[1]);
    // "/home/cyrus/workspace2/CrowdBotUnity/Scenario/CrowdBot/FollowBot/GeneratedScenario.xml"
    sim->setTime(0);

    Listener l(&node, sim);

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
