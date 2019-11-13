#ifndef CRAAL_SIMULATION_H_
#define CRAAL_SIMULATION_H_

#include "CraalIncludes.h"

#include "Configurable.h"
#include "PDF.h"

namespace craal {

/**
 * The base Simulation class which interfaces with crowd simulation algorithms.
 * A crowd simulation algorithm must extend this class in order to be usable
 * by the framework.
 * 
 * Suppose Model one such model and SimulationModel the class extending
 * @ref Simulation, then:
 *  - in SimulationModel's constructor, @ref Simulation::addParam() must be used
 *    to specify the model's parameters; the name of the model should be set
 *    (i.e. @code name = "Model"; @endcode )
 *  - implement @ref Simulation::addObstacleCoords() to be able to add obstacles
 *  - implement @ref Simulation::init() which initializes the model
 *  - implement @ref Simulation::reset() which copies parameter values from the
 *    framework to the implementation of Model; to avoid copying and thus to be
 *    more efficient, see comment in  @ref Simulation::reset()
 *  - implement @ref Simulation::setPosition() to set agents' positions
 *  - implement @ref Simulation::setVelocity() to set agents' velocities
 *  - implement @ref Simulation::setGoal() to set agents' goals
 *  - implement @ref Simulation::doStep() which computes the agents' next states
 * 
 * See @ref IMPLEMENT_SIM for examples on how to implement models.
 */
class Simulation : public Configurable
{
public:
	/**
	 * The PDF-type implementation of the @ref Configurable::Type class.
	 * Translates a @ref PDF into an std::string and vice-versa using the format
	 * "a b DIST c d" where:
	 *  - a is the lower bound of the PDF
	 *  - b is the higher bound of the PDF
	 *  - DIST is the distribution type:
	 *     - "UNIFORM" for a uniform distribution
	 *     - "NORMAL" for a normal distribution
	 *  - c is the mean when DIST is "NORMAL"
	 *  - d is the standard deviation when DIST is "NORMAL"
	 */
	class Type_pdf : public Configurable::Type
	{
	public:
		/**
		 * Translation: std::string => PDF
		 * @param s_str label of the attribute
		 * @param s_ptr void pointer to the PDF value
		 */
		virtual void assign(std::string s_str, void * s_ptr)
		{
			float a, b, c, d;
			PDF::DISTRIBUTION dist;
			char distr[16];
			
			int res = sscanf(s_str.c_str(), "%f %f %s %f %f", &a, &b, &distr, &c, &d);
			
			if (res >= 3) {
				if (std::string(distr).compare("UNIFORM") == 0) dist = PDF::UNIFORM;
				else if (std::string(distr).compare("NORMAL") == 0) dist = PDF::NORMAL;
			}
			
			switch (res)
			{
			case 2:
				*((PDF*)s_ptr) = PDF(a, b);
				break;
			case 3:
				*((PDF*)s_ptr) = PDF(a, b, dist);
				break;
			case 4:
				*((PDF*)s_ptr) = PDF(a, b, dist, c);
				break;
			case 5:
				*((PDF*)s_ptr) = PDF(a, b, dist, c, d);
				break;
			}
		};
		/**
		 * Translation: PDF => std::string
		 * @param s_ptr void pointer to the PDF value
		 * @return PDF value as std::string
		 */
		virtual std::string get(void * s_ptr)
		{
			std::stringstream ss;
			
			ss<< ((PDF*)s_ptr)->g_lowerBound;
			ss<< " ";
			ss<< ((PDF*)s_ptr)->g_higherBound;
			ss<< " ";
			
			switch (((PDF*)s_ptr)->g_distrib)
			{
				case PDF::UNIFORM:
					ss<< "UNIFORM";
					break;
				case PDF::NORMAL:
					ss<< "NORMAL";
					break;
			}
			ss<< " ";
			
			ss<< ((PDF*)s_ptr)->g_param1;
			ss<< " ";
			ss<< ((PDF*)s_ptr)->g_param2;
			
			return ss.str();
		};
	};

public:
	/** Timestep of the simulation. */
	float g_dt;
	/** Current time for the playback of the computed simulation (between 0 and 1). */
	float g_time;
	
	/** Number of pedestrians. */
	int g_nPedestrian;
	/** Number of obstacles. */
	int g_nObstacle;
	/** Number of computed frames. */
	int g_nMeasure;
	/** Number of parameters per agent. */
	int g_nbParams;
	
	/** Computed agents' X positions. */
	std::vector<float> * g_centerx;
	/** Computed agents' Y positions. */
	std::vector<float> * g_centery;
	/** Computed agents' X velocities. */
	std::vector<float> * g_centerVelocityx;
	/** Computed agents' Y velocities. */
	std::vector<float> * g_centerVelocityy;
	/** List of starting X coordinates of obstacles. */
	std::vector<float> g_obstaclesx;
	/** List of starting Y coordinates of obstacles. */
	std::vector<float> g_obstaclesy;
	/** List of ending X coordinates of obstacles. */
	std::vector<float> g_obstacleex;
	/** List of ending Y coordinates of obstacles. */
	std::vector<float> g_obstacleey;
	
	/** Agents' next computed X position coordinates (current frame). */
	float * g_centerxNext;
	/** Agents' next computed Y position coordinates (current frame). */
	float * g_centeryNext;
	/** Agents' next computed X velocity coordinates (current frame). */
	float * g_centerVelocityxNext;
	/** Agents' next computed Y velocity coordinates (current frame). */
	float * g_centerVelocityyNext;
	
	/** Array of all agents' parameter values. */
	float * g_params;
	/** The PDFs corresponding to the agents' parameters, size = @ref Simulation::g_nbParams. */
	std::vector<PDF> g_pdfs;
	/** The default values of the agents' parameters, size = @ref Simulation::g_nbParams. */
	std::vector<float> g_defaults;
	
	/** Delete all arrays. */
	void p_deleteArrays();

public:
	/** Constructor. */
	Simulation();
	/** Destructor. */
	virtual ~Simulation();
	
	/**
	 * Should be used to initialize the model's implementation.
	 * The @ref Simulation::g_nPedestrian attribute should be useful here.
	 */
	virtual void init() = 0;
	/**
	 * Used to copy the parameter values from the framework to the model's implementation.
	 * An agent's parameter value can be accessed through @ref Simulation::getParam().
	 * 
	 * Parameter copying happens when, for instance, agents' parameters are different attributes
	 * in a dedicated "Agent" class. A more efficient implementation, however, would be if
	 * the model stored the agents' parameters in an array where cell i*nbParams+j represents
	 * the value of agent i's parameter j.
	 * 
	 * I such a case, in @ref Simulation::init(), such an array should be made to point to
	 * @ref Simulation::getParam(0, 0). For example:
	 * @code MyModelSimulator->myArrayParams = getParam(0, 0); @endcode
	 */
	virtual void reset() = 0;
	
	/**
	 * Used to add obstacles.
	 * @param s_startx X coordinate of the obstacle's "start" vertex
	 * @param s_starty Y coordinate of the obstacle's "start" vertex
	 * @param s_endx X coordinate of the obstacle's "end" vertex
	 * @param s_endy Y coordinate of the obstacle's "end" vertex
	 */
	virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy) = 0;
	
	/**
	 * Used to set the agents' positions.
	 * @param s_indPedestrian index if the agent
	 * @param s_x X position coordinate
	 * @param s_y Y position coordinate
	 */
	virtual void setPosition(int s_indPedestrian, float s_x, float s_y) = 0;
	/**
	 * Used to set the agents' velocities.
	 * @param s_indPedestrian index if the agent
	 * @param s_x X velocity coordinate
	 * @param s_y Y velocity coordinate
	 */
	virtual void setVelocity(int s_indPedestrian, float s_x, float s_y) = 0;
	/**
	 * Used to set the agents' goals.
	 * @param s_indPedestrian index if the agent
	 * @param s_x X goal coordinate
	 * @param s_y Y goal coordinate
	 */
	virtual void setGoal(int s_indPedestrian, float s_x, float s_y) = 0;
	
	/**
	 * Used to compute the agents' next states.
	 * When the next velocities and positions have been computed, they should
	 * be stored using @ref Simulation::setNextState.
	 * @param s_dt the timestep of the simulation
	 */
	virtual void doStep(float s_dt = 0.01) = 0;
	
	/**
	 * Add a parameter to the model.
	 * This method:
	 *  - adds a new default value to @ref Simulation::g_defaults
	 *  - adds a new PDF to @ref Simulation::g_pdfs and sets it as a configurable attribute
	 *    (see @ref Configurable)
	 * @param s_name name of the parameter, will be used as the label of the configurable
	 * PDF attribute
	 * @param s_default default value for the parameter
	 * @param s_pdf the @ref PDF of the parameter
	 */
	void addParam(std::string s_name, float s_default, PDF s_pdf);
	/**
	 * Used to initialize the simulation.
	 * Among other things, calls the following methods in this order:
	 *  - @ref Simulation::init()
	 *  - @ref Simulation::reset()
	 * @param s_nPedestrian number of pedestrians
	 */
	virtual void initSimulation(int s_nPedestrian);
	/**
	 * Used to add obstacles.
	 * Calls @ref Simulation::addObstacleCoords()
	 * @param s_startx X coordinate of the obstacle's "start" vertex
	 * @param s_starty Y coordinate of the obstacle's "start" vertex
	 * @param s_endx X coordinate of the obstacle's "end" vertex
	 * @param s_endy Y coordinate of the obstacle's "end" vertex
	 */
	void addObstacle(float s_startx, float s_starty, float s_endx, float s_endy);
	/**
	 * Adds a new frame to the simulation for all agents.
	 * Uses values from:
	 *  - @ref Simulation::g_centerxNext
	 *  - @ref Simulation::g_centeryNext
	 *  - @ref Simulation::g_centerVelocityxNext
	 *  - @ref Simulation::g_centerVelocityyNext
	 */
	virtual void addState();
	/**
	 * Adds a new frame to the simulation for a given agent.
	 * @param s_indPedestrian index of the pedestrian
	 * @param s_px new position's X coordinate
	 * @param s_py new position's Y coordinate
	 * @param s_vx new velocity's X coordinate
	 * @param s_vy new velocity's Y coordinate
	 */
	void addState(int s_indPedestrian, float s_px, float s_py, float s_vx, float s_vy);
	/**
	 * Sets the next state of a given agent. Should be used by @ref Simulation::doStep().
	 * @param s_indPedestrian index of the pedestrian
	 * @param s_px new position's X coordinate
	 * @param s_py new position's Y coordinate
	 * @param s_vx new velocity's X coordinate
	 * @param s_vy new velocity's Y coordinate
	 */
	void setNextState(int s_indPedestrian, float s_px, float s_py, float s_vx, float s_vy);
	
	/** Erases all saved frames from the simulation. */
	virtual void clear();
	/** Updates the number of frames in the simulation @ref Simulation::g_nMeasure. */
	virtual void resetNMeasure();
	
	/**
	 * Get the list of PDFs.
	 */
	inline std::vector<PDF> getPdfs()
		{return g_pdfs;};
	
	/** Sets the number of parameters. */
	inline void setNParams(int s_nbParams)
		{g_nbParams = s_nbParams;};
	
	/**
	 * Get a parameter value.
	 * @param s_indPedestrian index of the agent
	 * @param s_indParam index of the parameter
	 * @return pointer to the parameter's value
	 */
	inline float * getParam(int s_indPedestrian, int s_indParam)
		{return g_params+s_indPedestrian*g_nbParams+s_indParam;};
	
	/**
	 * Set the time position for playback, usually used in conjunction with
	 * @ref Experiment::setTime().
	 * Determines which frame's data will be returned by
	 * @ref Simulation::getCenterx(int s_indPedestrian),
	 * @ref Simulation::getCentery(int s_indPedestrian),
	 * @ref Simulation::getCenterVelocityx(int s_indPedestrian) and
	 * @ref Simulation::getCenterVelocityy(int s_indPedestrian).
	 * @param s_time playback time position (between 0 and 1)
	 */
	inline void setTime(float s_time = 0) {g_time = s_time;};
	
	/** Get number of agents. */
	inline int getNPedestrian() {return g_nPedestrian;};
	/** Get number of obstacles. */
	inline int getNObstacle() {return g_nObstacle;};
	/** Get number of computed frames. */
	inline int getNMeasure() {return g_nMeasure;};
	/** Get number of paramters per agent. */
	inline int getNParam() {return g_nbParams;};
	/** Get the timestep. */
	inline float getDT() {return g_dt;};
    inline void setDT(float dt) {g_dt = dt;};
	
	/**
	 * Get the X coordinate of the starting vertex of an obstacle.
	 * @param s_obstacle index of the obstacle
	 */
	inline float getObstacleStartx(int s_obstacle)
		{return g_obstaclesx[s_obstacle];};
	/**
	 * Get the Y coordinate of the starting vertex of an obstacle.
	 * @param s_obstacle index of the obstacle
	 */
	inline float getObstacleStarty(int s_obstacle)
		{return g_obstaclesy[s_obstacle];};
	/**
	 * Get the X coordinate of the ending vertex of an obstacle.
	 * @param s_obstacle index of the obstacle
	 */
	inline float getObstacleEndx(int s_obstacle)
		{return g_obstacleex[s_obstacle];};
	/**
	 * Get the Y coordinate of the ending vertex of an obstacle.
	 * @param s_obstacle index of the obstacle
	 */
	inline float getObstacleEndy(int s_obstacle)
		{return g_obstacleey[s_obstacle];};
	
	/**
	 * Get the X position coordinate of an agent at the frame set by
	 * @ref Simulation::setTime().
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterx(int s_indPedestrian)
		{return g_centerx[s_indPedestrian][((int)(g_time*(g_nMeasure-1)))];};
	/**
	 * Get the Y position coordinate of an agent at the frame set by
	 * @ref Simulation::setTime().
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCentery(int s_indPedestrian)
		{return g_centery[s_indPedestrian][((int)(g_time*(g_nMeasure-1)))];};
	
	/**
	 * Get the X velocity coordinate of an agent at the frame set by
	 * @ref Simulation::setTime().
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterVelocityx(int s_indPedestrian)
		{return g_centerVelocityx[s_indPedestrian][((int)(g_time*(g_nMeasure-1)))];};
	/**
	 * Get the Y velocity coordinate of an agent at the frame set by
	 * @ref Simulation::setTime().
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterVelocityy(int s_indPedestrian)
		{return g_centerVelocityy[s_indPedestrian][((int)(g_time*(g_nMeasure-1)))];};
	
	/**
	 * Get the next X position coordinate of an agent.
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterxNext(int s_indPedestrian)
		{return g_centerxNext[s_indPedestrian];};
	/**
	 * Get the next Y position coordinate of an agent.
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenteryNext(int s_indPedestrian)
		{return g_centeryNext[s_indPedestrian];};
	
	/**
	 * Get the next X velocity coordinate of an agent.
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterVelocityxNext(int s_indPedestrian)
		{return g_centerVelocityxNext[s_indPedestrian];};
	/**
	 * Get the next Y velocity coordinate of an agent.
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterVelocityyNext(int s_indPedestrian)
		{return g_centerVelocityyNext[s_indPedestrian];};
	
	/**
	 * Get the X position coordinate of an agent at a given frame.
	 * @param s_indPedestrian index of the agent
	 * @param s_indMeasure index of the frame
	 */
	inline float getCenterx(int s_indPedestrian, int s_indMeasure)
		{return g_centerx[s_indPedestrian][s_indMeasure];};
	/**
	 * Get the Y position coordinate of an agent at a given frame.
	 * @param s_indPedestrian index of the agent
	 * @param s_indMeasure index of the frame
	 */
	inline float getCentery(int s_indPedestrian, int s_indMeasure)
		{return g_centery[s_indPedestrian][s_indMeasure];};
	
	/**
	 * Get the X velocity coordinate of an agent at a given frame.
	 * @param s_indPedestrian index of the agent
	 * @param s_indMeasure index of the frame
	 */
	inline float getCenterVelocityx(int s_indPedestrian, int s_indMeasure)
		{return g_centerVelocityx[s_indPedestrian][s_indMeasure];};
	/**
	 * Get the Y velocity coordinate of an agent at a given frame.
	 * @param s_indPedestrian index of the agent
	 * @param s_indMeasure index of the frame
	 */
	inline float getCenterVelocityy(int s_indPedestrian, int s_indMeasure)
		{return g_centerVelocityy[s_indPedestrian][s_indMeasure];};
};

}

#endif /* CRAAL_SIMULATION_H_ */
