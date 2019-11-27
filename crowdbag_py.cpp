#include <boost/python.hpp>
#include "crowdbag.hpp"
using namespace boost::python;

BOOST_PYTHON_MODULE(crowdsim)
{
    class_<CrowdSim>("CrowdSim", init<std::string>())
            .def("getNPedestrian", &CrowdSim::getNPedestrian)
            .def("getNObstacle", &CrowdSim::getNObstacle)

            .def("getObstacle", &CrowdSim::getObstacle)
            .def("getCenter", &CrowdSim::getCenter)
            .def("getCenterVelocity", &CrowdSim::getCenterVelocity)
            .def("getCenterNext", &CrowdSim::getCenterNext)
            .def("getCenterVelocityNext", &CrowdSim::getCenterVelocityNext)

            .def("getObstacleStartx", &CrowdSim::getObstacleStartx)
            .def("getObstacleStarty", &CrowdSim::getObstacleStarty)
            .def("getObstacleEndx", &CrowdSim::getObstacleEndx)
            .def("getObstacleEndy", &CrowdSim::getObstacleEndy)
            .def("getCenterx", &CrowdSim::getCenterx)
            .def("getCentery", &CrowdSim::getCentery)
            .def("getCenterVelocityx", &CrowdSim::getCenterVelocityx)
            .def("getCenterVelocityy", &CrowdSim::getCenterVelocityy)
            .def("getCenterxNext", &CrowdSim::getCenterxNext)
            .def("getCenteryNext", &CrowdSim::getCenteryNext)
            .def("getCenterVelocityxNext", &CrowdSim::getCenterVelocityxNext)
            .def("getCenterVelocityyNext", &CrowdSim::getCenterVelocityyNext)

            .def("init", &CrowdSim::init)
            .def("doStep", &CrowdSim::doStep)
            .def("setTime", &CrowdSim::setTime)
            .def("setGoal", &CrowdSim::setGoal)
            .def("setPosition", &CrowdSim::setPosition)
            .def("setVelocity", &CrowdSim::setVelocity)
            .def("addObstacleCoords", &CrowdSim::addObstacleCoords)

            .def("setAgentRadius", &CrowdSim::setAgentRadius)
            .def("setAgentSpeed", &CrowdSim::setAgentSpeed)
            .def("setAgentNeighborDist", &CrowdSim::setAgentNeighborDist)
            .def("setAgentTimeHorizon", &CrowdSim::setAgentTimeHorizon)

            .def("getPi", &CrowdSim::getPi)
    ;
};
