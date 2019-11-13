#ifndef CRAAL_EXPERIMENT_H_
#define CRAAL_EXPERIMENT_H_

#include "CraalIncludes.h"

#include "Parser.h"

namespace craal {

/**
 * The Experiment class which stores the reference data.
 * Data input is done through objects which extend the @ref Parser class.
 */
class Experiment
{
protected:
	float g_time;
	
	int g_nPedestrian;
	int g_nObstacle;
	int g_nMeasure;
	
	float *g_centerx, *g_centery;
	float *g_centerVelocityx, *g_centerVelocityy;
	float * g_times;
	
	float *g_obstaclesx, *g_obstaclesy, *g_obstacleex, *g_obstacleey;
	
	bool g_realData;
	int * g_nWaypoint;
	float **g_waypointx, **g_waypointy;
	float **g_waypointSize;
	
	int g_onlySimulateNb;
	int * g_onlySimulate;

public:
	/** Constructor. */
	Experiment();
	/** Constructor, calls @ref Experiment::init(). */
	Experiment(Parser & s_parser);
	/** Destructor. */
	virtual ~Experiment();
	
	/**
	 *Transfers data from the @ref Parser object to the Experiment.
	 * @param s_parser input @ref Parser object
	 */
	virtual void init(Parser & s_parser);
	
    /** Get number of obstacles. */
    inline int getNObstacle() {return g_nObstacle;};
	/** Get number of agents. */
    virtual inline int getNPedestrian() {return g_nPedestrian;};
	/** Get number of frames. */
    virtual inline int getNMeasure() {return g_nMeasure;};
	/** Get the total duration of the reeference situation. */
    virtual inline float getDuration(){return g_times[g_nMeasure-1];};
	/** Get the number of agents that are simulated. **/
    virtual inline float getNSimulated(){return g_onlySimulateNb;};
	
	/**
	 * Tells if the data consists of recorded trajectories.
	 * A false returned value indicates that the data consists of waypoints
	 * the agents should reach (useful for instance when using fundamental
	 * diagrams which store the reference data themselves).
	 */
	inline bool isRealData() {return g_realData;};
	
	/**
	 * Set the time position for playback, usually used in conjunction with
	 * @ref Simulation::setTime().
	 * Determines which frame's data will be returned by
	 * @ref Experiment::getCenterx(int s_ped), @ref Experiment::getCentery(int s_ped),
	 * @ref Experiment::getCenterVelocityx(int s_ped) and @ref Experiment::getCenterVelocityy(int s_ped).
	 * @param s_time playback time position (between 0 and 1)
	 */
     virtual inline void setTime(float s_time = 0) {g_time = s_time;};
	
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
	 * Get the index of a simulated agent.
	 * @param s_index index of the agent
	 */
	inline int getSimulatedIndex(int s_index)
		{return g_onlySimulate[s_index];};
	
	/**
	 * Get the X position coordinate of an agent at the frame set by
	 * @ref Experiment::setTime().
	 *@param s_ped index of the agent 
	 */
	virtual inline float getCenterx(int s_ped = 0)
		{return g_centerx[s_ped*g_nMeasure+((int)(g_time*(g_nMeasure-1)))];};
	/**
	 * Get the Y position coordinate of an agent at the frame set by
	 * @ref Experiment::setTime().
	 *@param s_ped index of the agent 
	 */
	virtual inline float getCentery(int s_ped = 0)
		{return g_centery[s_ped*g_nMeasure+((int)(g_time*(g_nMeasure-1)))];};
	
	/**
	 * Get the X position coordinate of an agent at a given frame.
	 * @param s_ped index of the agent
	 * @param s_indMeasure index of the frame
	 */
	virtual inline float getCenterx(int s_ped, int s_indMeasure)
		{return g_centerx[s_ped*g_nMeasure+s_indMeasure];};
	/**
	 * Get the Y position coordinate of an agent at a given frame.
	 * @param s_ped index of the agent
	 * @param s_indMeasure index of the frame
	 */
	virtual inline float getCentery(int s_ped, int s_indMeasure)
		{return g_centery[s_ped*g_nMeasure+s_indMeasure];};
	
	/**
	 * Get the X velocity coordinate of an agent at the frame set by
	 * @ref Experiment::setTime().
	 *@param s_ped index of the agent 
	 */
	virtual inline float getCenterVelocityx(int s_ped = 0)
		{return g_centerVelocityx[s_ped*g_nMeasure+((int)(g_time*(g_nMeasure-1)))];};
	/**
	 * Get the Y velocity coordinate of an agent at the frame set by
	 * @ref Experiment::setTime().
	 *@param s_ped index of the agent 
	 */
	virtual inline float getCenterVelocityy(int s_ped = 0)
		{return g_centerVelocityy[s_ped*g_nMeasure+((int)(g_time*(g_nMeasure-1)))];};
	
	/**
	 * Get the X velocity coordinate of an agent at a given frame.
	 * @param s_ped index of the agent
	 * @param s_indMeasure index of the frame
	 */
	virtual inline float getCenterVelocityx(int s_ped, int s_indMeasure)
		{return g_centerVelocityx[s_ped*g_nMeasure+s_indMeasure];};
	/**
	 * Get the Y velocity coordinate of an agent at a given frame.
	 * @param s_ped index of the agent
	 * @param s_indMeasure index of the frame
	 */
	virtual inline float getCenterVelocityy(int s_ped, int s_indMeasure)
		{return g_centerVelocityy[s_ped*g_nMeasure+s_indMeasure];};
	
	/**
	 * Get the time at a given frame since the beginning.
	 * @param s_indMeasure the index of the frame
	 * @return time since the beginning
	 */
    virtual inline float getTime(int s_indMeasure)
		{return g_times[s_indMeasure];};
	
	/**
	 * Get the number of waypoints attributed to a given agent.
	 * @param s_ped the index of the agent
	 * @return the agent's number of waypoints
	 */
	inline int getNWaypoint(int s_ped)
		{return g_nWaypoint[s_ped];};
	/**
	 * Get the radius of a waypoint.
	 * @param s_ped the index of the agent
	 * @param s_waypoint the index of the waypoint
	 * @return the radius of the waypoint
	 */
	inline int getWaypointSize(int s_ped, int s_waypoint)
		{return g_waypointSize[s_ped][s_waypoint];};
	
	/**
	 * Get the X position coordinate of an agent's waypoint.
	 * @param s_ped the index of the agent
	 * @param s_waypoint the index of the waypoint
	 */
	inline float getWaypointx(int s_ped, int s_waypoint)
		{return g_waypointx[s_ped][s_waypoint];};
	/**
	 * Get the Y position coordinate of an agent's waypoint.
	 * @param s_ped the index of the agent
	 * @param s_waypoint the index of the waypoint
	 */
	inline float getWaypointy(int s_ped, int s_waypoint)
		{return g_waypointy[s_ped][s_waypoint];};
};

}

#endif /* CRAAL_EXPERIMENT_H_ */
