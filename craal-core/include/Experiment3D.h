#ifndef CRAAL_EXPERIMENT3D_H_
#define CRAAL_EXPERIMENT3D_H_

#include "Experiment.h"
#include "Parser3D.h"

namespace craal {

/**
 * The 3D version of @ref Experiment.
 */
class Experiment3D : public Experiment
{
protected:
	float *g_centerz;
	float *g_centerVelocityz;
	
	float **g_waypointz;

public:
	/** Constructor. */
	Experiment3D();
	/** Constructor, calls @ref Experiment::init(). */
	Experiment3D(Parser3D & s_parser);
	/** Destructor. */
	virtual ~Experiment3D();
	
	/**
	 *Transfers data from the @ref Parser object to the Experiment.
	 * @param s_parser input @ref Parser object
	 */
	virtual void init(Parser3D & s_parser);
	
	/**
	 * Get the Z position coordinate of an agent at the frame set by
	 * @ref Experiment::setTime().
	 *@param s_ped index of the agent 
	 */
	virtual inline float getCenterz(int s_ped = 0)
		{return g_centerz[s_ped*g_nMeasure+((int)(g_time*(g_nMeasure-1)))];};
	
	/**
	 * Get the Z position coordinate of an agent at a given frame.
	 * @param s_ped index of the agent
	 * @param s_indMeasure index of the frame
	 */
	virtual inline float getCenterz(int s_ped, int s_indMeasure)
		{return g_centerz[s_ped*g_nMeasure+s_indMeasure];};
	
	/**
	 * Get the Z velocity coordinate of an agent at the frame set by
	 * @ref Experiment::setTime().
	 *@param s_ped index of the agent 
	 */
	virtual inline float getCenterVelocityz(int s_ped = 0)
		{return g_centerVelocityz[s_ped*g_nMeasure+((int)(g_time*(g_nMeasure-1)))];};
	
	/**
	 * Get the Z velocity coordinate of an agent at a given frame.
	 * @param s_ped index of the agent
	 * @param s_indMeasure index of the frame
	 */
	virtual inline float getCenterVelocityz(int s_ped, int s_indMeasure)
		{return g_centerVelocityz[s_ped*g_nMeasure+s_indMeasure];};
	
	/**
	 * Get the Z position coordinate of an agent's waypoint.
	 * @param s_ped the index of the agent
	 * @param s_waypoint the index of the waypoint
	 */
	inline float getWaypointz(int s_ped, int s_waypoint)
		{return g_waypointz[s_ped][s_waypoint];};
};

}

#endif /* CRAAL_EXPERIMENT3D_H_ */
