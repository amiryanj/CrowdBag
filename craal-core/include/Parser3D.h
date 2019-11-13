#ifndef CRAAL_PARSER3D_H_
#define CRAAL_PARSER3D_H_

#include "Parser.h"

namespace craal {

/**
 * The 3D version of @ref Parser.
 */
class Parser3D : public Parser
{
public:
	/** Constructor. */
	Parser3D(){};
	/** Destructor. */
	virtual ~Parser3D(){};
	
	/**
	 * Read data from a file.
	 * @param s_filename path to the file
	 */
	virtual void parse(const char * s_filename) = 0;
	
	/**
	 * Used by @ref Experiment objects to transfer the data.
	 * This method should fill the variables pointed to by its parameters:
	 * @param s_nPed (*s_nPed) should be the number of agents
	 * @param s_nObs (*s_nPed) should be the number of obstacles
	 * @param s_nMes (*s_nPed) should be the number of frames
	 * @param s_centerx (*s_centerx)[i*s_nMes+j] should be the X position coordinate
	 * of agent i at frame j
	 * @param s_centery (*s_centery)[i*s_nMes+j] should be the Y position coordinate
	 * of agent i at frame j
	 * @param s_centerz (*s_centerz)[i*s_nMes+j] should be the Z position coordinate
	 * of agent i at frame j
	 * @param s_centerVelocityx (*s_centerVelocityx)[i*s_nMes+j] should be the X velocity coordinate
	 * of agent i at frame j
	 * @param s_centerVelocityy (*s_centerVelocityy)[i*s_nMes+j] should be the Y velocity coordinate
	 * of agent i at frame j
	 * @param s_centerVelocityz (*s_centerVelocityz)[i*s_nMes+j] should be the Z velocity coordinate
	 * of agent i at frame j
	 * @param s_obstacleStartx (*s_obstacleStartx)[i] should be the X position of obstacle i's
	 * start vertex
	 * @param s_obstacleStarty (*s_obstacleStarty)[i] should be the Y position of obstacle i's
	 * start vertex
	 * @param s_obstacleEndx (*s_obstacleEndx)[i] should be the X position of obstacle i's
	 * end vertex
	 * @param s_obstacleEndy (*s_obstacleEndy)[i] should be the Y position of obstacle i's
	 * end vertex
	 * @param s_times (*s_times)[i] should be the time at frame i since the beginning
	 * @param s_realData (*s_realData) should be the flag indicating if the data is real or fake
	 * (see @ref Experiment::isRealData())
	 * @param s_nWaypoint (*s_nWaypoint)[i] should be the number of waypoints of agent i
	 * @param s_waypointSize (*s_waypointSize)[i][j] should be the radius of agent i's waypoint j
	 * @param s_waypointx (*s_waypointx)[i][j] should be the X position coordinate of agent i's waypoint j
	 * @param s_waypointy (*s_waypointy)[i][j] should be the Y position coordinate of agent i's waypoint j
	 * @param s_waypointz (*s_waypointz)[i][j] should be the Z position coordinate of agent i's waypoint j
	 * @param s_onlySimulateNb number of simulated agents
	 * @param s_onlySimulate indices of simulated agents
	 */
	virtual void fill(int * s_nPed, int * s_nObs, int * s_nMes,
		float ** s_centerx, float ** s_centery, float ** s_centerz,
		float ** s_centerVelocityx, float ** s_centerVelocityy, float ** s_centerVelocityz,
		float ** s_obstacleStartx, float ** s_obstacleStarty,
		float ** s_obstacleEndx, float ** s_obstacleEndy,
		float ** s_times,
		bool * s_realData, int ** s_nWaypoint, float *** s_waypointSize, 
		float *** s_waypointx, float *** s_waypointy, float *** s_waypointz,
		int * s_onlySimulateNb, int ** s_onlySimulate) = 0;
		
	virtual void fill(int * s_nPed, int * s_nObs, int * s_nMes,
		float ** s_centerx, float ** s_centery,
		float ** s_centerVelocityx, float ** s_centerVelocityy,
		float ** s_obstacleStartx, float ** s_obstacleStarty,
		float ** s_obstacleEndx, float ** s_obstacleEndy,
		float ** s_times,
		bool * s_realData, int ** s_nWaypoint, float *** s_waypointSize, 
		float *** s_waypointx, float *** s_waypointy,
		int * s_onlySimulateNb, int ** s_onlySimulate)
	{};
};

}

#endif /* CRAAL_PARSER3D_H_ */
