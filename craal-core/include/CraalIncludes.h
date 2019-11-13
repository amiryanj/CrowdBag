#ifndef CRAAL_INCLUDES_H_
#define CRAAL_INCLUDES_H_

#include <algorithm>
#include <cstring>
#include <deque>
#include <fstream>
#include <iostream>
#include <map>
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <time.h>
#include <vector>

#define CRAAL_MAX(a,b) (((a)>=(b))?(a):(b))
#define CRAAL_MIN(a,b) (((a)<=(b))?(a):(b))

#define CRAAL_PI 3.14159265

#ifdef _WIN32

#define isnan(x) _isnan(x)
#define isinf(x) (!_finite(x))

#endif /* _WIN32 */

#endif /* CRAAL_INCLUDES_H_ */

namespace craal {

/**
 * @mainpage
 * 
 * The framework is organized as follows:
 *  - a @b core directory containing all the classes that are necessary
 *    as well as the base classes for:
 *     - parsers (@ref Parser)
 *     - models (@ref Simulation)
 *     - metrics (@ref Metric)
 *     - calibration methods (@ref Calibration)
 *  - a @b parsers directory containing implementations of the @ref Parser
 *    class dealing with various data file types
 *  - a @b models directory containing implementations of the @ref Simulation
 *    class which interface the framework with various models
 *  - a @b metrics directory containing implementations of the @ref Metric
 *    class
 *  - a @b calibration directory containing the implementations of the
 *    @ref Calibration class
 *  - a @b graphics directory containing the code for the GUI, requires
 *    qt5, OpenGL and freeglut
 *  - a @b batch directory containing the code for batch-processing
 * 
 * Apart from the graphics which require qt5, OpenGL and freeglut, the framework doesn't
 * rely on any particular libraries and it is possible to choose which parts should be
 * built through the cmake interface.
 * 
 * To add a new implementation of a parser/model/metric/calibration method to the framework,
 * the implementation should extend the correct base class and be placed in the corresponding
 * directory. Any new dependencies are then to be added to the corresponding "-addional.cmake"
 * file. See @ref ADD_IMPLEMENTATION for examples on adding implementations to the framework.
 * 
 * As provided, the framework can be used through a graphical user interface (see @ref GUIU),
 * batch-processing (see @ref BATCH) or its core components (see @ref CUSTOM).
 * All programs can use the framework through the following "CMakeLists.txt" file:
 * 
 * @code
 * 
 * cmake_minimum_required(VERSION 2.8.8)
 * 
 * # Project
 * project(Example)
 * 
 * # Craal
 * find_package(craal REQUIRED)
 * set(LIBRARIES ${CRAAL_LIBRARIES})
 * 
 * # Source Files
 * file(
 *     GLOB
 *     CPP_SOURCES
 *     src/main.cpp
 * )
 * 
 * # Output
 * add_executable(
 *     test
 *     ${CPP_SOURCES}
 * )
 * 
 * # Linking
 * target_link_libraries(test ${LIBRARIES})
 * 
 * @endcode
 */

/**
 * @page CUSTOM Custom usage
 * 
 * Example of a custom usage of the framework:
 * @code
 * 
 * #include <iostream>
 * 
 * #include "Genetic.h"
 * #include "Difference.h"
 * #include "ParserCSV.h"
 * #include "Record.h"
 * #include "SimulationModel.h"
 * 
 * using namespace craal;
 * 
 * int main(int argc, char ** argv)
 * {
 *     ParserCSV parser;
 *     parser.parse("somefile.csv");
 *     
 *     Experiment exp(parser);
 *     SimulationModel sim;
 *     
 *     Record record;
 *     record.setExperiment(&exp);
 *     record.setSimulation(&sim);
 *     record.initSimulation();
 *     
 *     Difference metric;
 *     float initialScore = metric.evaluate(&record);
 *     
 *     Genetic calibration;
 *     calibration.setMetric(&metric);
 *     calibration.setRecord(&record);
 *     calibration.verbose(true);
 *     
 *     calibration.calibrate();
 *     
 *     float calScore = calibration.getScore();
 *     
 *     std::cout<< "Initial score: "<< initialScore<< std::endl;
 *     std::cout<< "Score after calibration: "<< calScore<< std::endl;
 *     std::cout<< "Parameters: "<< std::endl;
 *     calibration.printParams(std::cout);
 *     
 *     return 0;
 * }
 * 
 * @endcode
 */

/**
 * @page GUIU GUI usage
 * 
 * Example of a GUI usage of the framework (using the @ref Application class):
 * 
 * @code
 * 
 * #include "Application.h"
 * 
 * #include "ParserANA.h"
 * #include "ParserCSV.h"
 * 
 * #include "SimulationModel1.h"
 * #include "SimulationModel2.h"
 * 
 * #include "Difference.h"
 * #include "ProgressiveDifference.h"
 * 
 * #include "Genetic.h"
 * #include "Greedy.h"
 * 
 * using namespace craal;
 * 
 * class GeneticGreedy : public Genetic, public Greedy
 * {
 * public:
 *     GeneticGreedy(){name = "GeneticGreedy";};
 *     virtual void calibrate()
 *     {
 *         Genetic::calibrate();
 *         Greedy::calibrate();
 *     };
 * };
 * 
 * int main(int argc, char ** argv)
 * {
 *     ResourceLister::addParser<ParserANA>();
 *     ResourceLister::addParser<ParserCSV>();
 * 
 *     ResourceLister::addSimulation<SimulationModel1>();
 *     ResourceLister::addSimulation<SimulationModel2>();
 * 
 *     ResourceLister::addMetric<Difference>();
 *     ResourceLister::addMetric<ProgressiveDifference>();
 * 
 *     ResourceLister::addCalibration<Greedy>();
 *     ResourceLister::addCalibration<Genetic>();
 *     ResourceLister::addCalibration<GeneticGreedy>();
 * 
 *     return Application::run(argc, argv);
 * }
 * 
 * @endcode
 */

/**
 * @page BATCH Batch testing usage
 * 
 * Example of a batch-testing usage of the framework (using the @ref BatchProcessing class):
 * 
 * @code
 * 
 * #include "BatchProcessing.h"
 * 
 * #include "ParserANA.h"
 * #include "ParserCSV.h"
 * 
 * #include "SimulationModel1.h"
 * #include "SimulationModel2.h"
 * 
 * #include "Difference.h"
 * #include "ProgressiveDifference.h"
 * 
 * #include "Genetic.h"
 * #include "Greedy.h"
 * 
 * using namespace craal;
 * 
 * class GeneticGreedy : public Genetic, public Greedy
 * {
 * public:
 *     GeneticGreedy(){name = "GeneticGreedy";};
 *     virtual void calibrate()
 *     {
 *         Genetic::calibrate();
 *         Greedy::calibrate();
 *     };
 * };
 * 
 * BatchProcessing batch;
 * 
 * void setBMet()
 * {
 *     std::cout<< ""<< batch.ifile+1<< "/5 "<< batch.file<< std::endl;
 * }
 * 
 * void setBMod()
 * {
 *     std::cout<< "-"<< batch.imetric+1<< "/2 "<< batch.metric->name<< std::endl;
 * }
 * 
 * void setBCal()
 * {
 *     std::cout<< "--"<< batch.imodel+1<< "/2 "<< batch.model->name<< std::endl;
 * }
 * 
 * void setBTry()
 * {
 *     std::cout<< "---"<< batch.icalib+1<< "/3 "<< batch.calibration->name<< std::endl;
 * }
 * 
 * void setOnTry()
 * {
 *     std::cout<< "----"<< batch.itry+1<< "/"<< 3<< std::endl;
 *     std::cout<< "     Score before: "<< batch.scoreBefore<< std::endl;
 *     std::cout<< "     Score after: "<< batch.scoreAfter<< std::endl;
 *     std::cout<< "     Duration: "<< batch.time<< std::endl;
 * }
 * 
 * int main(int argc, char ** argv)
 * {
 *     batch.addFile("file1.csv");
 *     batch.addFile("file2.csv");
 *     batch.addFile("file3.ana");
 *     batch.addFile("file4.csv");
 *     batch.addFile("file5.ana");
 * 
 *     ResourceLister::addParser<ParserANA>();
 *     ResourceLister::addParser<ParserCSV>();
 * 
 *     ResourceLister::addSimulation<SimulationModel1>();
 *     ResourceLister::addSimulation<SimulationModel2>();
 * 
 *     ResourceLister::addMetric<Difference>();
 *     ResourceLister::addMetric<ProgressiveDifference>();
 * 
 *     ResourceLister::addCalibration<Greedy>();
 *     ResourceLister::addCalibration<Genetic>();
 *     ResourceLister::addCalibration<GeneticGreedy>();
 * 
 *     batch.setTries(3);
 * 
 *     batch.setBeforeMetric(setBMet);
 *     batch.setBeforeModel(setBMod);
 *     batch.setBeforeCalib(setBCal);
 *     batch.setBeforeTry(setBTry);
 *     batch.setOnTry(setOnTry);
 * 
 *     batch.process();
 * 
 *     return 0;
 * }
 * 
 * @endcode
 */

/**
 * @page ADD_IMPLEMENTATION Adding implementations
 * 
 * The "clean way" of adding implementations of parsers/models/metrics/calibration methods to the
 * framework is done in 3 steps:
 *  - extend the corresponding base class (see @ref IMPLEMENT_PARSER, @ref IMPLEMENT_SIM,
 *    @ref IMPLEMENT_METRIC and @ref IMPLEMENT_CALIB)
 *  - add the implementation to the correct directory
 *  - update the corresponding "CMakeLists.txt" file and specify the additional dependencies
 *    in the corresponding "-additional.cmake" file
 * 
 * For instance, if a model implemented in a library SomeModel is to be added to the framework,
 * a class SomeModel should extend the base class @ref Simulation (see @ref IMPLEMENT_SIM) and
 * be put in the "models" folder.
 * 
 * The "CMakeLists.txt" should look as follows:
 * 
 * @code
 * 
 * cmake_minimum_required(VERSION 2.8.8)
 * 
 * # Project
 * project(Models)
 * 
 * include(models-additional.cmake)
 * 
 * # Includes
 * include_directories(include)
 * include_directories(../core/include)
 * 
 * # Headers
 * set(MODELS_HEADERS
 *     ${CMAKE_CURRENT_LIST_DIR}/include/SomeModel.h     # <-- the new class goes here
 *     PARENT_SCOPE
 * )
 * 
 * # Source Files
 * file(
 *     GLOB
 *     CPP_SOURCES
 *     src/SomeModel.cpp                                 # <-- the new class implementation goes here
 * )
 * 
 * # Output
 * add_library(
 *     models OBJECT
 *     ${CPP_SOURCES}
 * )
 * 
 * @endcode
 * 
 * And the following should be appended to the "models-additional.cmake" file:
 * 
 *  - case where SomeModel is compiled with cmake:
 * @code
 * # Some model
 * find_package(somemodel REQUIRED)
 * set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} ${SOMEMODEL_LIBRARIES})
 * @endcode
 * 
 *  - other case:
 * @code
 * # Some model
 * include_directories(/path/to/somemodel/headers/)
 * link_directories(/path/to/somemodel/library/)
 * set(CRAAL_MODELS_LIBRARIES ${CRAAL_MODELS_LIBRARIES} somemodel)
 * @endcode
 */

/**
 * @page IMPLEMENT_PARSER Implementing a parser
 * 
 * This page shows an example of a parser implementation.
 * 
 * @code
 * 
 * #################################### SomeParser.h ####################################
 * 
 * #ifndef SOMEPARSER_H_
 * #define SOMEPARSER_H_
 * 
 * #include "Parser.h"
 * 
 * namespace craal {
 * 
 * class SomeParser : public Parser
 * {
 * private:
 *     int g_nped;
 *     int g_nobs;
 *     int g_nmes;
 *     float *g_centerx, *g_centery;
 *     float *g_centerVelocityx, *g_centerVelocityy;
 *     float *g_obstacleStartx, *g_obstacleStarty;
 *     float *g_obstacleEndx, *g_obstacleEndy;
 *     float *g_times;
 *     bool g_realData;
 * 
 * public:
 *     SomeParser();
 *     virtual SomeParser();
 *     
 *     virtual void parse(const char * s_filename);
 *     
 *     virtual void fill(int * s_nPed, int * s_nObs, int * s_nMes,
 *         float ** s_centerx, float ** s_centery,
 *         float ** s_centerVelocityx, float ** s_centerVelocityy,
 *         float ** s_obstacleStartx, float ** s_obstacleStarty,
 *         float ** s_obstacleEndx, float ** s_obstacleEndy,
 *         float ** s_times,
 *         bool * s_realData, int ** s_nWaypoint, float *** s_waypointSize, 
 *         float *** s_waypointx, float *** s_waypointy);
 * };
 * 
 * }
 * 
 * #endif // SOMEPARSER_H_
 * 
 * #################################### SomeParser.cpp ####################################
 * 
 * #include "SomeParser.h"
 * 
 * namespace craal {
 * 
 * SomeParser::SomeParser() :
 *     g_nped(0), g_nobs(0), g_nmes(0),
 *     g_centerx(0), g_centery(0), g_centerVelocityx(0), g_centerVelocityy(0),
 *     g_obstacleStartx(0), g_obstacleStarty(0), g_obstacleEndx(0), g_obstacleEndy(0),
 *     g_times(0), g_realData(0)
 * {
 *     g_extension = "ext";
 * }
 * 
 * SomeParser::SomeParser()
 * {
 *     if (g_centerx) delete [] g_centerx;
 *     if (g_centery) delete [] g_centery;
 *     if (g_centerVelocityx) delete [] g_centerVelocityx;
 *     if (g_centerVelocityy) delete [] g_centerVelocityy;
 *     if (g_obstacleStartx) delete [] g_obstacleStartx;
 *     if (g_obstacleStarty) delete [] g_obstacleStarty;
 *     if (g_obstacleEndx) delete [] g_obstacleEndx;
 *     if (g_obstacleEndy) delete [] g_obstacleEndy;
 *     if (g_times) delete [] g_times;
 * }
 * 
 * void SomeParser::fill(int * s_nPed, int * s_nObs, int * s_nMes,
 *         float ** s_centerx, float ** s_centery,
 *         float ** s_centerVelocityx, float ** s_centerVelocityy,
 *         float ** s_obstacleStartx, float ** s_obstacleStarty,
 *         float ** s_obstacleEndx, float ** s_obstacleEndy,
 *         float ** s_times,
 *         bool * s_realData, int ** s_nWaypoint, float *** s_waypointSize, 
 *         float *** s_waypointx, float *** s_waypointy)
 * {
 *     (*s_nPed) = g_nped;
 *     (*s_nObs) = g_nobs;
 *     (*s_nMes) = g_nmes;
 *     (*s_centerx) = g_centerx;
 *     (*s_centery) = g_centery;
 *     (*s_centerVelocityx) = g_centerVelocityx;
 *     (*s_centerVelocityy) = g_centerVelocityy;
 *     (*s_obstacleStartx) = g_obstacleStartx;
 *     (*s_obstacleStarty) = g_obstacleStarty;
 *     (*s_obstacleEndx) = g_obstacleEndx;
 *     (*s_obstacleEndy) = g_obstacleEndy;
 *     (*s_times) = g_times;
 *     (*s_realData) = g_realData;
 *     
 *     g_centerx = 0;
 *     g_centery = 0;
 *     g_centerVelocityx = 0;
 *     g_centerVelocityy = 0;
 *     g_obstacleStartx = 0;
 *     g_obstacleStarty = 0;
 *     g_obstacleEndx = 0;
 *     g_obstacleEndy = 0;
 *     g_times = 0;
 * }
 * 
 * void SomeParser::parse(const char * s_fileName)
 * {
 *     // open file s_fileName, read some data
 * 
 *     int g_nped = 5;
 *     int g_nobs = 4;
 *     int g_nmes = 500;
 *     bool g_realData = true;
 *     
 *     // fill some values
 *     
 *     g_centerx = new float[g_nped*g_nmes];
 *     g_centery = new float[g_nped*g_nmes];
 *     
 *     g_centerVelocityx = new float[g_nped*g_nmes];
 *     g_centerVelocityy = new float[g_nped*g_nmes];
 *     
 *     g_obstacleStartx = new float[g_nobs];
 *     g_obstacleStarty = new float[g_nobs];
 *     g_obstacleEndx = new float[g_nobs];
 *     g_obstacleEndy = new float[g_nobs];
 *     
 *     g_times = new float[nmes];
 *     
 *     // fill some more values
 * }
 * 
 * }
 * 
 * @endcode
 */

/**
 * @page IMPLEMENT_SIM Implementing a simulation
 * 
 * This page shows an example of a model implementation.
 * 
 * @code
 * 
 * #################################### SomeModel.h ####################################
 * 
 * #ifndef SOMEMODEL_H_
 * #define SOMEMODEL_H_
 * 
 * #include "Simulation.h"
 * #include "SomeModelSimulator.h"
 * 
 * namespace craal {
 * 
 * class SomeModel : public Simulation
 * {
 * private:
 *     SomeModelSimulator * g_sim;
 * 
 * public:
 *     SomeModel();
 *     virtual ~SomeModel();
 * 
 *     virtual void reset();
 * 
 *     virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);
 *     
 *     virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
 *     virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
 *     virtual void setGoal(int s_indPedestrian, float s_x, float s_y);
 *     
 *     virtual void doStep(float s_dt = 0.01);
 * };
 * 
 * }
 * 
 * #endif // SOMEMODEL_H_
 * 
 * #################################### SomeModel.cpp ####################################
 * 
 * #include "SomeModel.h"
 * 
 * namespace craal {
 * 
 * SomeModel::SomeModel() :
 *     g_sim(0)
 * {
 *     addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
 *     addParam("Radius", 15.0, PDF(10, 20, PDF::NORMAL, 15, 5));
 *     
 *     name = "SomeModel";
 * }
 * 
 * void SomeModel::~SomeModel()
 * {
 *     if (g_sim) delete g_sim;
 * }
 * 
 * void SomeModel::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
 * {
 *     g_sim->addObstacle(s_startx, s_starty, s_endx, s_endy);
 * }
 * 
 * void SomeModel::init()
 * {
 *     if (g_sim) delete g_sim;
 *     
 *     g_sim = new SomeModelSimulator();
 *     
 *     for (int i = 0; i < g_nPedestrian; i++) {
 *         g_sim->addAgent();
 *     }
 * 
 *     // start of optional code: optimized implementation (see comment in Simulation::reset())
 *     g_sim->myArrayParams = getParam(0, 0);
 *     // end of optional code
 * }
 * 
 * void SomeModel::reset()
 * {
 *     for (int i = 0; i < g_nPedestrian; i++) {
 *         g_sim->setAgentSpeed(i, g_params[i*2+0]);
 *         g_sim->setAgentRadius(i, g_params[i*2+1]);
 *     }
 * 
 *     // optional: in an optimized implementation (see comment in Simulation::reset()),
 *     // this method should do nothing
 * }
 * 
 * void SomeModel::setPosition(int s_indPedestrian, float s_x, float s_y)
 * {
 *     g_sim->setAgentPosition(s_indPedestrian, s_x, s_y);
 * }
 * 
 * void SomeModel::setVelocity(int s_indPedestrian, float s_x, float s_y)
 * {
 *     g_sim->setAgentVelocity(s_indPedestrian, s_x, s_y);
 * }
 * 
 * void SomeModel::setGoal(int s_indPedestrian, float s_x, float s_y)
 * {
 *     g_sim->setAgentVelocity(s_indPedestrian, s_x, s_y);
 * }
 * 
 * void SomeModel::doStep(float s_dt)
 * {
 *     g_sim->doStep(s_dt);
 *     
 *     for (int i = 0; i < g_nPedestrian; i++) {
 *         setNextState(i,
 *             g_sim->getAgentPosition(i).x(),
 *             g_sim->getAgentPosition(i).y(),
 *             g_sim->getAgentVelocity(i).x(),
 *             g_sim->getAgentVelocity(i).y());
 *     }
 * }
 * 
 * }
 * 
 * @endcode
 */

/**
 * @page IMPLEMENT_METRIC Implementing a metric
 * 
 * This page shows two example metric implementations as well as a combined metric.
 * 
 * SomeMetric:
 * 
 * @code
 * 
 * #################################### SomeMetric.h ####################################
 * 
 * #ifndef SOMEMETRIC_H_
 * #define SOMEMETRIC_H_
 * 
 * #include "Metric.h"
 * 
 * namespace craal {
 * 
 * class SomeMetric : virtual public Metric
 * {
 * public:
 *     SomeMetric();
 *     virtual ~SomeMetric();
 * 
 *     virtual float evaluate(Record * s_record);
 * };
 * 
 * }
 * 
 * #endif // SOMEMETRIC_H_
 * 
 * #################################### SomeMetric.cpp ####################################
 * 
 * #include "SomeMetric.h"
 * 
 * namespace craal {
 * 
 * SomeMetric::SomeMetric()
 * {
 *     name = "SomeMetric";
 * }
 * 
 * SomeMetric::~SomeMetric()
 * {}
 * 
 * float SomeMetric::evaluate(Record * s_record)
 * {
 *     // compute the complete paths
 *     s_record->computePath();
 * 
 *     float result;
 *     
 *     // do stuff
 *     
 *     return result;
 * }
 * 
 * }
 * 
 * @endcode
 * 
 * SomeOtherMetric:
 * 
 * @code
 * 
 * #################################### SomeOtherMetric.h ####################################
 * 
 * #ifndef SOMEOTHERMETRIC_H_
 * #define SOMEOTHERMETRIC_H_
 * 
 * #include "Metric.h"
 * 
 * namespace craal {
 * 
 * class SomeOtherMetric : virtual public Metric
 * {
 * public:
 *     SomeOtherMetric();
 *     virtual SomeOtherMetric();
 * 
 *     virtual float evaluate(Record * s_record);
 * };
 * 
 * }
 * 
 * #endif // SOMEOTHERMETRIC_H_
 * 
 * #################################### SomeOtherMetric.cpp ####################################
 * 
 * #include "SomeOtherMetric.h"
 * 
 * namespace craal {
 * 
 * SomeOtherMetric::SomeOtherMetric()
 * {
 *     name = "SomeOtherMetric";
 * }
 * 
 * SomeOtherMetric::SomeOtherMetric()
 * {}
 * 
 * float SomeOtherMetric::evaluate(Record * s_record)
 * {
 *     float result;
 *     
 *     // do stuff
 * 
 *     // for some reason we only consider the third agent's decision at the 5th frame...
 *     // reposition him in the simulator:
 *     float x, y;
 *     float vx, vy;
 * 
 *     x = s_record->getExpPositionxSimulated(2, 4);
 *     y = s_record->getExpPositionySimulated(2, 4);
 * 
 *     vx = s_record->getExpVelocityxSimulated(2, 4);
 *     vy = s_record->getExpVelocityySimulated(2, 4);
 * 
 *     s_record->setSimPositionSimulated(2, x, y);
 *     s_record->setSimVelocitySimulated(2, vx, vy);
 * 
 *     // compute the next state:
 *     s_record->computeNext();
 * 
 *     // do some other stuff
 *     
 *     return result;
 * }
 * 
 * }
 * 
 * @endcode
 * 
 * The combination of SomeMetric and SomeOtherMetric:
 * 
 * @code
 * 
 * class Combined : public SomeMetric, public SomeOtherMetric
 * {
 * public:
 *     float weight1, weight2;
 * 
 *     Combined()
 *     {
 *         name = "SomeMetric+SomeOtherMetric";
 * 
 *         weight1 = 0.3;
 *         weight2 = 0.7;
 * 
 *         // we later want to be able to change the weights
 *         addConfigurable("Weight1", new Type_float, &weight1);
 *         addConfigurable("Weight2", new Type_float, &weight2);
 *     };
 *     virtual float evaluate(Record * s_record)
 *     {
 *         return weight1*SomeMetric::evaluate(s_record) + weight2*SomeOtherMetric::evaluate(s_record);
 *     };
 * };
 * 
 * @endcode
 */

/**
 * @page IMPLEMENT_CALIB Implementing a calibration algorithm
 * 
 * This page shows an example of a calibration algorithm as well as combined one.
 * 
 * SomeCalibration:
 * 
 * @code
 * 
 * #################################### SomeCalibration.h ####################################
 * 
 * #ifndef SOMECALIBRATION_H_
 * #define SOMECALIBRATION_H_
 * 
 * #include "Calibration.h"
 * 
 * namespace craal {
 * 
 * class SomeCalibration : virtual public Calibration
 * {
 * public:
 *     SomeCalibration();
 *     virtual ~SomeCalibration();
 * 
 *     virtual void calibrate();
 * };
 * 
 * }
 * 
 * #endif // SOMECALIBRATION_H_
 * 
 * #################################### SomeCalibration.cpp ####################################
 * 
 * #include "SomeCalibration.h"
 * 
 * namespace craal {
 * 
 * SomeCalibration::SomeCalibration()
 * {
 *     name = "SomeCalibration";
 * };
 * 
 * SomeCalibration::~SomeCalibration()
 * {}
 * 
 * void SomeCalibration::calibrate()
 * {
 *     // get initial score
 *     g_record->resetSimulation();
 *     float initScore = g_metric->evaluate(g_record);
 * 
 *     // do stuff, change parameters
 *     
 *     // apply parameters, recompute score
 *     g_record->resetSimulation();
 *     float newScore = g_metric->evaluate(g_record);
 *     
 *     // change parameters some more etc...
 * 
 *     // found a minimum, save best score
 *     g_record->resetSimulation();
 *     g_score = g_metric->evaluate(g_record);
 * }
 * 
 * }
 * 
 * @endcode
 * 
 * Combining SomeCalibration and SomeOtherCalibration:
 * 
 * @code
 * 
 * class CombinedCalibration : public SomeCalibration, public SomeOtherCalibration
 * {
 * public:
 *     CombinedCalibration(){name = "SomeCalibration+SomeOtherCalibration";};
 *     virtual void calibrate()
 *     {
 *         SomeCalibration::calibrate();
 *         SomeOtherCalibration::calibrate();
 *     };
 * };
 * 
 * @endcode
 */

}
