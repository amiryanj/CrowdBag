#include <iostream>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include "rosgraph_msgs/Clock.h"
#include "./xml_loader.hpp"
//#include "./crowdbag.hpp"

#include "crowdbag/SimulateCrowd.h"
#include "crowdbag/SimPedestrianState.h"
#include "crowdbag/CrowdState.h"

#include <memory>


using namespace std;


class SimulatorHandler
{
    CrowdSim *sim;
public:
    SimulatorHandler(const char * xml_file) {
        this->sim = XMLLoader().load(xml_file);
        sim->setTime(0);
    }

    bool simulate(crowdbag::SimulateCrowd::Request  &req,
                  crowdbag::SimulateCrowd::Response  &res)
    {
        for(int i=0; i<req.cur_state.peds.size(); i++) {
            sim->setPosition(i, req.cur_state.peds[i].position.x, req.cur_state.peds[i].position.y);
            sim->setVelocity(i, req.cur_state.peds[i].velocity.x, req.cur_state.peds[i].velocity.y);
            sim->setGoal(i, req.cur_state.peds[i].goal.x, req.cur_state.peds[i].goal.y);
        }
        sim->doStep(req.dt);

        for(int i=0; i<req.cur_state.peds.size(); i++) {
            crowdbag::SimPedestrianState ped_i;
            float pix = sim->getCenterxNext(i);
            float piy = sim->getCenteryNext(i);
            float vix = sim->getCenterVelocityxNext(i);
            float viy = sim->getCenterVelocityyNext(i);
            ped_i.position.x = pix;
            ped_i.position.y = piy;
            ped_i.velocity.x = vix;
            ped_i.velocity.y = viy;
            res.next_state.peds.push_back(ped_i);
        }
    }
};

bool foo(crowdbag::SimulateCrowd::Request  &req, crowdbag::SimulateCrowd::Response  &res) {

}

int main(int argc, char **argv)
{
  if(argc < 2) {
        cerr << "Error! please set the xml file as input!" << endl;
        exit(0);
  }
  ros::init(argc, argv, "crowdbag_server");
  ros::NodeHandle node;

  SimulatorHandler sim_handler(argv[1]);
  ros::ServiceServer service = node.advertiseService("crowdbag", foo);
  ROS_INFO("Ready to simulate crowd.");
  ros::spin();

  return 0;
}








