#ifndef CRAAL_RECORD3D_H_
#define CRAAL_RECORD3D_H_

#include "Experiment3D.h"
#include "Record.h"
#include "Simulation3D.h"

namespace craal {

/**
 * The 3D version of @ref Record.
 */
class Record3D : public Record
{
protected:
	/** 
	 * Set starting positions and velocities as well as the goals of the agents in
	 * @ref Record::g_simulation.
	 */
	virtual void p_initSim();
	/** Computes trajectories in the case of real data (see @ref Experiment::isRealData()). */
	virtual void p_computePathRealData();
	/**
	 *Computes trajectories in the case of fake data (see @ref Experiment::isRealData()).
	 * @todo remove s_save flag used for debugging.
	 */
	virtual void p_computePathFakeData(bool s_save = false);

public:
	/** Constructor. */
	Record3D();
	/** Destructor. */
	virtual ~Record3D();
	
	/**
	 * Saves the trajectories from @ref Record::g_simulation.
	 * @param s_filename filename
	 */
	virtual void save(const char * s_filename);
	
	/**
	 * Get the Z position coordinate of an agent from @ref Record::g_experiment at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return Z position coordinate
	 */
	inline float getExpPositionz(int s_iPed, int s_iMes)
		{return ((Experiment3D*)g_experiment)->getCenterz(s_iPed, s_iMes);}
	/**
	 * Get the Z velocity coordinate of an agent from @ref Record::g_experiment at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return Z velocity coordinate
	 */
	inline float getExpVelocityz(int s_iPed, int s_iMes)
		{return ((Experiment3D*)g_experiment)->getCenterVelocityz(s_iPed, s_iMes);}
	
	/**
	 * Get the Z position coordinate of an agent from @ref Record::g_experiment at the currrent time.
	 * @param s_iPed index of the agent
	 * @return Z position coordinate
	 */
	inline float getExpPositionz(int s_iPed)
		{return ((Experiment3D*)g_experiment)->getCenterz(s_iPed);}
	/**
	 * Get the Z velocity coordinate of an agent from @ref Record::g_experiment at the currrent time.
	 * @param s_iPed index of the agent
	 * @return Z velocity coordinate
	 */
	inline float getExpVelocityz(int s_iPed)
		{return ((Experiment3D*)g_experiment)->getCenterVelocityz(s_iPed);}
	
	/**
	 * Get the Z position coordinate of an agent from @ref Record::g_simulation at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_iMes index of the frame
	 * @return Z position coordinate
	 */
	inline float getSimPositionz(int s_iPed, int s_iMes)
		{return ((Simulation3D*)g_simulation)->getCenterz(g_map[s_iPed], s_iMes);}
	/**
	 * Get the Z velocity coordinate of an agent from @ref Record::g_simulation at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_iMes index of the frame
	 * @return Z velocity coordinate
	 */
	inline float getSimVelocityz(int s_iPed, int s_iMes)
		{return ((Simulation3D*)g_simulation)->getCenterVelocityz(g_map[s_iPed], s_iMes);}
	
	/**
	 * Get the Z position coordinate of an agent from @ref Record::g_simulation at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return Z position coordinate
	 */
	inline float getSimPositionz(int s_iPed)
		{return ((Simulation3D*)g_simulation)->getCenterz(g_map[s_iPed]);}
	/**
	 * Get the Z velocity coordinate of an agent from @ref Record::g_simulation at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return Z velocity coordinate
	 */
	inline float getSimVelocityz(int s_iPed)
		{return ((Simulation3D*)g_simulation)->getCenterVelocityz(g_map[s_iPed]);}
	
	/**
	 * Get the next Z position coordinate of an agent from @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return next Z position coordinate
	 */
	inline float getSimPositionzNext(int s_iPed)
		{return ((Simulation3D*)g_simulation)->getCenterzNext(g_map[s_iPed]);}
	/**
	 * Get the next Z velocity coordinate of an agent from @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return next Z velocity coordinate
	 */
	inline float getSimVelocityzNext(int s_iPed)
		{return ((Simulation3D*)g_simulation)->getCenterVelocityzNext(g_map[s_iPed]);}
	
	/**
	 * Get the Z position coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @param s_iMes index of the frame
	 * @return Z position coordinate
	 */
	inline float getExpPositionzSimulated(int s_iPed, int s_iMes)
		{return ((Experiment3D*)g_simulation)->getCenterz(g_invmap[s_iPed], s_iMes);}
	/**
	 * Get the Z velocity coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @param s_iMes index of the frame
	 * @return Z velocity coordinate
	 */
	inline float getExpVelocityzSimulated(int s_iPed, int s_iMes)
		{return ((Experiment3D*)g_experiment)->getCenterVelocityz(g_invmap[s_iPed], s_iMes);}
	
	/**
	 * Get the Z position coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @return Z position coordinate
	 */
	inline float getExpPositionzSimulated(int s_iPed)
		{return ((Experiment3D*)g_experiment)->getCenterz(g_invmap[s_iPed]);}
	/**
	 * Get the Z velocity coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @return Z velocity coordinate
	 */
	inline float getExpVelocityzSimulated(int s_iPed)
		{return ((Experiment3D*)g_experiment)->getCenterVelocityz(g_invmap[s_iPed]);}
	
	/**
	 * Get the Z position coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return Z position coordinate
	 */
	inline float getSimPositionzSimulated(int s_iPed, int s_iMes)
		{return ((Simulation3D*)g_simulation)->getCenterz(s_iPed, s_iMes);}
	/**
	 * Get the Z velocity coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return Z velocity coordinate
	 */
	inline float getSimVelocityzSimulated(int s_iPed, int s_iMes)
		{return ((Simulation3D*)g_simulation)->getCenterVelocityz(s_iPed, s_iMes);}
	
	/**
	 * Get the Z position coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at the current time.
	 * @param s_iPed index of the agent
	 * @return Z position coordinate
	 */
	inline float getSimPositionzSimulated(int s_iPed)
		{return ((Simulation3D*)g_simulation)->getCenterz(s_iPed);}
	/**
	 * Get the Z velocity coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at the current time.
	 * @param s_iPed index of the agent
	 * @return Z velocity coordinate
	 */
	inline float getSimVelocityzSimulated(int s_iPed)
		{return ((Simulation3D*)g_simulation)->getCenterVelocityz(s_iPed);}
	
	/**
	 * Get the next Z position coordinate of an agent (simulated) from @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @return next Z position coordinate
	 */
	inline float getSimPositionzNextSimulated(int s_iPed)
		{return ((Simulation3D*)g_simulation)->getCenterzNext(s_iPed);}
	/**
	 * Get the next Z velocity coordinate of an agent (simulated) from @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @return next Z velocity coordinate
	 */
	inline float getSimVelocityzNextSimulated(int s_iPed)
		{return ((Simulation3D*)g_simulation)->getCenterVelocityzNext(s_iPed);}
	
	/**
	 * Set the position of an agent in @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_x X position coordinate
	 * @param s_y Y position coordinate
	 * @param s_y Z position coordinate
	 */
	inline void setSimPosition(int s_iPed, float s_x, float s_y, float s_z)
		{((Simulation3D*)g_simulation)->setPosition(g_map[s_iPed], s_x, s_y, s_z);}
	/**
	 * Set the velocity of an agent in @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_x X velocity coordinate
	 * @param s_y Y velocity coordinate
	 * @param s_y Z velocity coordinate
	 */
	inline void setSimVelocity(int s_iPed, float s_x, float s_y, float s_z)
		{((Simulation3D*)g_simulation)->setVelocity(g_map[s_iPed], s_x, s_y, s_z);}
	/**
	 * Set the goal of an agent in @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_x X goal coordinate
	 * @param s_y Y goal coordinate
	 * @param s_y Z goal coordinate
	 */
	inline void setSimGoal(int s_iPed, float s_x, float s_y, float s_z)
		{((Simulation3D*)g_simulation)->setGoal(g_map[s_iPed], s_x, s_y, s_z);}
	
	/**
	 * Set the position of an agent (simulated) in @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @param s_x X position coordinate
	 * @param s_y Y position coordinate
	 * @param s_y Z position coordinate
	 */
	inline void setSimPositionSimulated(int s_iPed, float s_x, float s_y, float s_z)
		{((Simulation3D*)g_simulation)->setPosition(s_iPed, s_x, s_y, s_z);}
	/**
	 * Set the velocity of an agent (simulated) in @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @param s_x X velocity coordinate
	 * @param s_y Y velocity coordinate
	 * @param s_y Z velocity coordinate
	 */
	inline void setSimVelocitySimulated(int s_iPed, float s_x, float s_y, float s_z)
		{((Simulation3D*)g_simulation)->setVelocity(s_iPed, s_x, s_y, s_z);}
	/**
	 * Set the goal of an agent (simulated) in @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @param s_x X goal coordinate
	 * @param s_y Y goal coordinate
	 * @param s_y Z goal coordinate
	 */
	inline void setSimGoalSimulated(int s_iPed, float s_x, float s_y, float s_z)
		{((Simulation3D*)g_simulation)->setGoal(s_iPed, s_x, s_y, s_z);}
};

}

#endif /* CRAAL_RECORD3D_H_ */
