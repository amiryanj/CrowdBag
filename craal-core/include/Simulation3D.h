#ifndef CRAAL_SIMULATION3D_H_
#define CRAAL_SIMULATION3D_H_

#include "Simulation.h"

namespace craal {

/**
 * The 3D version of @ref Simulation.
 */
class Simulation3D : public Simulation
{
protected:
	/** Computed agents' Z positions. */
	std::vector<float> * g_centerz;
	/** Computed agents' Z velocities. */
	std::vector<float> * g_centerVelocityz;
	
	/** Agents' next computed X position coordinates (current frame). */
	float * g_centerzNext;
	/** Agents' next computed X velocity coordinates (current frame). */
	float * g_centerVelocityzNext;
	
	/** Delete all arrays. */
	void p_deleteArrays();

public:
	/** Constructor. */
	Simulation3D();
	/** Destructor. */
	virtual ~Simulation3D();
	
	/**
	 * Used to set the agents' positions.
	 * @param s_indPedestrian index if the agent
	 * @param s_x X position coordinate
	 * @param s_y Y position coordinate
	 * @param s_z Z position coordinate
	 */
	virtual void setPosition(int s_indPedestrian, float s_x, float s_y, float s_z) = 0;
	virtual void setPosition(int s_indPedestrian, float s_x, float s_y)
	{};
	/**
	 * Used to set the agents' velocities.
	 * @param s_indPedestrian index if the agent
	 * @param s_x X velocity coordinate
	 * @param s_y Y velocity coordinate
	 * @param s_z Z velocity coordinate
	 */
	virtual void setVelocity(int s_indPedestrian, float s_x, float s_y, float s_z) = 0;
	virtual void setVelocity(int s_indPedestrian, float s_x, float s_y)
	{};
	/**
	 * Used to set the agents' goals.
	 * @param s_indPedestrian index if the agent
	 * @param s_x X goal coordinate
	 * @param s_y Y goal coordinate
	 * @param s_z Z goal coordinate
	 */
	virtual void setGoal(int s_indPedestrian, float s_x, float s_y, float s_z) = 0;
	virtual void setGoal(int s_indPedestrian, float s_x, float s_y)
	{};
	
	/**
	 * Used to initialize the simulation.
	 * Among other things, calls the following methods in this order:
	 *  - @ref Simulation::init()
	 *  - @ref Simulation::reset()
	 * @param s_nPedestrian number of pedestrians
	 */
	virtual void initSimulation(int s_nPedestrian);
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
	 * @param s_pz new position's Z coordinate
	 * @param s_vx new velocity's X coordinate
	 * @param s_vy new velocity's Y coordinate
	 * @param s_vz new velocity's Z coordinate
	 */
	void addState(int s_indPedestrian,
		float s_px, float s_py, float s_pz,
		float s_vx, float s_vy, float s_vz);
	/**
	 * Sets the next state of a given agent. Should be used by @ref Simulation::doStep().
	 * @param s_indPedestrian index of the pedestrian
	 * @param s_px new position's X coordinate
	 * @param s_py new position's Y coordinate
	 * @param s_pz new position's Z coordinate
	 * @param s_vx new velocity's X coordinate
	 * @param s_vy new velocity's Y coordinate
	 * @param s_vz new velocity's Z coordinate
	 */
	void setNextState(int s_indPedestrian,
		float s_px, float s_py, float s_pz,
		float s_vx, float s_vy, float s_vz);
	
	/** Erases all saved frames from the simulation. */
	virtual void clear();
	/** Updates the number of frames in the simulation @ref Simulation::g_nMeasure. */
	virtual void resetNMeasure();
	
	/**
	 * Get the Z position coordinate of an agent at the frame set by
	 * @ref Simulation::setTime().
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterz(int s_indPedestrian)
		{return g_centerz[s_indPedestrian][((int)(g_time*(g_nMeasure-1)))];};
	
	/**
	 * Get the Z velocity coordinate of an agent at the frame set by
	 * @ref Simulation::setTime().
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterVelocityz(int s_indPedestrian)
		{return g_centerVelocityz[s_indPedestrian][((int)(g_time*(g_nMeasure-1)))];};
	
	/**
	 * Get the next Z position coordinate of an agent.
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterzNext(int s_indPedestrian)
		{return g_centerzNext[s_indPedestrian];};
	
	/**
	 * Get the next Z velocity coordinate of an agent.
	 *@param s_indPedestrian index of the agent 
	 */
	inline float getCenterVelocityzNext(int s_indPedestrian)
		{return g_centerVelocityzNext[s_indPedestrian];};
	
	/**
	 * Get the Z position coordinate of an agent at a given frame.
	 * @param s_indPedestrian index of the agent
	 * @param s_indMeasure index of the frame
	 */
	inline float getCenterz(int s_indPedestrian, int s_indMeasure)
		{return g_centerz[s_indPedestrian][s_indMeasure];};
	
	/**
	 * Get the Z velocity coordinate of an agent at a given frame.
	 * @param s_indPedestrian index of the agent
	 * @param s_indMeasure index of the frame
	 */
	inline float getCenterVelocityz(int s_indPedestrian, int s_indMeasure)
		{return g_centerVelocityz[s_indPedestrian][s_indMeasure];};
};

}

#endif /* CRAAL_SIMULATION3D_H_ */
