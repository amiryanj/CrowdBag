#ifndef CRAAL_PARSER_H_
#define CRAAL_PARSER_H_

#include "CraalIncludes.h"

namespace craal {

/**
 * The base Parser class. Any parser must extend this class in order to
 * be usable by the framework.
 * This class defines @ref Parser::g_extension
 * which specifies the extension of files supported by the parser and which
 * must be set in the constructor (e.g. @code g_extension = "csv"; @endcode ).
 * The extended class must also implement:
 * - the @ref Parser::parse() method which is used to read data from a file,
 * - the @ref Parser::fill() method which is used by @ref Experiment objects
 *   to transfer the data.
 * 
 * See @ref IMPLEMENT_PARSER for examples on how to implement parsers.
 */
class Parser
{
protected:
	/** Represents the extension of supported files. */
	std::string g_extension;
    bool allow_variant_lenghts = false;

public:
	/** Constructor. */
	Parser(){};
	/** Destructor. */
	virtual ~Parser(){};
	
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
	 * @param s_centerVelocityx (*s_centerVelocityx)[i*s_nMes+j] should be the X velocity coordinate
	 * of agent i at frame j
	 * @param s_centerVelocityy (*s_centerVelocityy)[i*s_nMes+j] should be the Y velocity coordinate
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
	 * @param s_onlySimulateNb number of simulated agents
	 * @param s_onlySimulate indices of simulated agents
	 */
	virtual void fill(int * s_nPed, int * s_nObs, int * s_nMes,
		float ** s_centerx, float ** s_centery,
		float ** s_centerVelocityx, float ** s_centerVelocityy,
		float ** s_obstacleStartx, float ** s_obstacleStarty,
		float ** s_obstacleEndx, float ** s_obstacleEndy,
		float ** s_times,
		bool * s_realData, int ** s_nWaypoint, float *** s_waypointSize, 
		float *** s_waypointx, float *** s_waypointy,
        int * s_onlySimulateNb, int ** s_onlySimulate) = 0;

    virtual void getNonSymmericMetaData(int ** s_centerCumulativeLenghts,
                                    int ** s_centerStartFrames ) {};

	
	/** Returns the extension of files handled by the parser. */
	inline std::string getExension(){return g_extension;}
    inline bool getAllowVariantLenghts() {return allow_variant_lenghts;}
    inline void setAllowVariantLenghts(bool allow) {allow_variant_lenghts = allow;}
};



}

#endif /* CRAAL_PARSER_H_ */
