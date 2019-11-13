#ifndef CRAAL_RECORD_H_
#define CRAAL_RECORD_H_

#include "CraalIncludes.h"

#include "Configurable.h"
#include "ConfigurationParser.h"
#include "Experiment.h"
#include "Simulation.h"

namespace craal {

/**
 * The Record class which contains and synchronizes an @ref Experiment object
 * and a @ref Simulation object.
 * It acts like a proxy between the two objects and parts of the code that
 * would draw information from them. One case is when not all pedestrians
 * have simulated counterparts that should be taken care of by the 
 * simulator. In that case, the agents that are controlled by the simulator
 * occupy the lowest indices in the @ref Simulation object; the Record
 * object then handles the mapping with the pedestrians from the
 * @ref Experiment object (which are not re-ordered).
 */
class Record : public Configurable
{
protected:
	/** The @ref Experiment object. */
	Experiment * g_experiment;
	/** The @ref Simulation object. */
	Simulation * g_simulation;
	
	/** Array of booleans telling, for each agent whether it is controlled by the simulator or not. */
	bool * g_mask;
	/** Maps @ref Record::g_experiment pedestrians to @ref Record::g_simulation agents. */
	int * g_map;
	/** Maps @ref Record::g_simulation pedestrians to @ref Record::g_experiment agents. */
	int * g_invmap;
	
	/** Number of agents. */
	int g_nPedestrian;
	/** Number of agents that are controlled by the simulator. */
	int g_nPedestrianSimulated;
	/** Number of frames in @ref Record::g_experiment object. */
	int g_nMeasure;
	/** Number of parameters per agent in @ref Record::g_simulation. */
	int g_nParams;
	
	/** Array of agent parameters, points to @ref Simulation::getParam(0, 0). */
	float * g_params;
	
	/**
	 * Filepath where to save trajectories from @ref Record::g_params
	 * (@ref Configurable attribute).
	 */
	std::string g_saveTrajectories;
	/**
	 * Filepath where to save parameter values from @ref Record::g_params
	 * (@ref Configurable attribute).
	 */
	std::string g_saveParameters;
	/**
	 * Filepath where to load parameter values from (into @ref Record::g_params)
	 * (@ref Configurable attribute).
	 */
	std::string g_loadParameters;
	
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
	Record();
	/** Destructor. */
	virtual ~Record();
	
	/** Initializes the @ref Record::g_simulation object. */
	virtual void initSimulation();
	/** Resets the simulation: calls @ref Simulation::reset() and @ref Record::p_initSim(). */
	virtual void resetSimulation();
	/**
	 * Sets if an agent should be controlled by the simulator.
	 * After being called on all agents that should be affected, @ref Record::saveMask() should
	 * be called.
	 * @param s_indPedestrian index of the agent
	 * @param s_simulate floag indicating if the agent should be simulated
	 */
	void setMask(int s_indPedestrian, bool s_simulate);
	/**
	 * Saves all changes made by @ref Record::setMask().
	 */
	void saveMask();
	
	/**
	 * Calls @ref Simulation::doStep() with the timestep from @ref Record::g_experiment.
	 */
	void computeNext();
	/**
	 * Computes trajectories, decides between @ref Record::p_computePathFakeData()
	 * and @ref Record::p_computePathRealData().
	 * @todo remove s_save flag used for debugging.
	 */
	virtual void computePath(bool s_save = false);
	
	/**
	 * @todo Currently does nothing...
	 */
	void syncSimulation(float s_time);
	/**
	 * Calls both @ref Experiment::setTime() and @ref Simulation::setTime().
	 * @param s_time time position applied to @ref Experiment::setTime() and
	 * @ref Simulation::setTime(), between 0 and 1
	 */
	void sync(float s_time);
	
	/**
	 * Saves the trajectories from @ref Record::g_simulation.
	 * @param s_filename filename
	 */
	virtual void save(const char * s_filename);
	/**
	 * Saves the parameters from @ref Record::g_simulation to a file (hexfloat format).
	 * @param s_filename file to write
	 */
	void saveParams(const char * s_filename);
	/**
	 * Reads the parameters from a file (hexfloat format) to @ref Record::g_simulation.
	 * @param s_filename file to read
	 */
	void loadParams(const char * s_filename);
	
	/**
	 * Implemented from @ref Configurable::updateSpecific().
	 * Calls:
	 *  - @ref Record::save() with @ref Record::g_saveTrajectories
	 *  - @ref Record::saveParams() with @ref Record::g_saveParameters
	 *  - @ref Record::loadParams() with @ref Record::g_loadParameters
	 * if any of these @ref Configurable attributes have changed.
	 */
	virtual void updateSpecific();
	
	/** Returns the total number of agents. */
	inline int getNPedestrian()
		{return g_nPedestrian;};
	/** Returns the number of agents that are controlled by the simulator. */
	inline int getNPedestrianSimulated()
		{return g_nPedestrianSimulated;};
	/** Returns the number of frames. */
	inline int getNMeasure()
		{return g_nMeasure;};
	
	/** Equivalent to @ref Simulation::getParam(). */
	inline float * getParam(int s_iPed, int s_iParam)
		{return g_params+s_iPed*g_nParams+s_iParam;};
	
	/**
	 * Get the X position coordinate of an agent from @ref Record::g_experiment at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return X position coordinate
	 */
	inline float getExpPositionx(int s_iPed, int s_iMes)
		{return g_experiment->getCenterx(s_iPed, s_iMes);}
	/**
	 * Get the Y position coordinate of an agent from @ref Record::g_experiment at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return Y position coordinate
	 */
	inline float getExpPositiony(int s_iPed, int s_iMes)
		{return g_experiment->getCentery(s_iPed, s_iMes);}
	/**
	 * Get the X velocity coordinate of an agent from @ref Record::g_experiment at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return X velocity coordinate
	 */
	inline float getExpVelocityx(int s_iPed, int s_iMes)
		{return g_experiment->getCenterVelocityx(s_iPed, s_iMes);}
	/**
	 * Get the Y velocity coordinate of an agent from @ref Record::g_experiment at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return Y velocity coordinate
	 */
	inline float getExpVelocityy(int s_iPed, int s_iMes)
		{return g_experiment->getCenterVelocityy(s_iPed, s_iMes);}
	
	/**
	 * Get the X position coordinate of an agent from @ref Record::g_experiment at the currrent time.
	 * @param s_iPed index of the agent
	 * @return X position coordinate
	 */
	inline float getExpPositionx(int s_iPed)
		{return g_experiment->getCenterx(s_iPed);}
	/**
	 * Get the Y position coordinate of an agent from @ref Record::g_experiment at the currrent time.
	 * @param s_iPed index of the agent
	 * @return Y position coordinate
	 */
	inline float getExpPositiony(int s_iPed)
		{return g_experiment->getCentery(s_iPed);}
	/**
	 * Get the X velocity coordinate of an agent from @ref Record::g_experiment at the currrent time.
	 * @param s_iPed index of the agent
	 * @return X velocity coordinate
	 */
	inline float getExpVelocityx(int s_iPed)
		{return g_experiment->getCenterVelocityx(s_iPed);}
	/**
	 * Get the Y velocity coordinate of an agent from @ref Record::g_experiment at the currrent time.
	 * @param s_iPed index of the agent
	 * @return Y velocity coordinate
	 */
	inline float getExpVelocityy(int s_iPed)
		{return g_experiment->getCenterVelocityy(s_iPed);}
	
	/**
	 * Get the X position coordinate of an agent from @ref Record::g_simulation at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_iMes index of the frame
	 * @return X position coordinate
	 */
	inline float getSimPositionx(int s_iPed, int s_iMes)
		{return g_simulation->getCenterx(g_map[s_iPed], s_iMes);}
	/**
	 * Get the Y position coordinate of an agent from @ref Record::g_simulation at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_iMes index of the frame
	 * @return Y position coordinate
	 */
	inline float getSimPositiony(int s_iPed, int s_iMes)
		{return g_simulation->getCentery(g_map[s_iPed], s_iMes);}
	/**
	 * Get the X velocity coordinate of an agent from @ref Record::g_simulation at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_iMes index of the frame
	 * @return X velocity coordinate
	 */
	inline float getSimVelocityx(int s_iPed, int s_iMes)
		{return g_simulation->getCenterVelocityx(g_map[s_iPed], s_iMes);}
	/**
	 * Get the Y velocity coordinate of an agent from @ref Record::g_simulation at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_iMes index of the frame
	 * @return Y velocity coordinate
	 */
	inline float getSimVelocityy(int s_iPed, int s_iMes)
		{return g_simulation->getCenterVelocityy(g_map[s_iPed], s_iMes);}
	
	/**
	 * Get the X position coordinate of an agent from @ref Record::g_simulation at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return X position coordinate
	 */
	inline float getSimPositionx(int s_iPed)
		{return g_simulation->getCenterx(g_map[s_iPed]);}
	/**
	 * Get the Y position coordinate of an agent from @ref Record::g_simulation at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return Y position coordinate
	 */
	inline float getSimPositiony(int s_iPed)
		{return g_simulation->getCentery(g_map[s_iPed]);}
	/**
	 * Get the X velocity coordinate of an agent from @ref Record::g_simulation at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return X velocity coordinate
	 */
	inline float getSimVelocityx(int s_iPed)
		{return g_simulation->getCenterVelocityx(g_map[s_iPed]);}
	/**
	 * Get the Y velocity coordinate of an agent from @ref Record::g_simulation at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return Y velocity coordinate
	 */
	inline float getSimVelocityy(int s_iPed)
		{return g_simulation->getCenterVelocityy(g_map[s_iPed]);}
	
	/**
	 * Get the next X position coordinate of an agent from @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return next X position coordinate
	 */
	inline float getSimPositionxNext(int s_iPed)
		{return g_simulation->getCenterxNext(g_map[s_iPed]);}
	/**
	 * Get the next Y position coordinate of an agent from @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return next Y position coordinate
	 */
	inline float getSimPositionyNext(int s_iPed)
		{return g_simulation->getCenteryNext(g_map[s_iPed]);}
	/**
	 * Get the next X velocity coordinate of an agent from @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return next X velocity coordinate
	 */
	inline float getSimVelocityxNext(int s_iPed)
		{return g_simulation->getCenterVelocityxNext(g_map[s_iPed]);}
	/**
	 * Get the next Y velocity coordinate of an agent from @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @return next Y velocity coordinate
	 */
	inline float getSimVelocityyNext(int s_iPed)
		{return g_simulation->getCenterVelocityyNext(g_map[s_iPed]);}
	
	/**
	 * Get the X position coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @param s_iMes index of the frame
	 * @return X position coordinate
	 */
	inline float getExpPositionxSimulated(int s_iPed, int s_iMes)
		{return g_experiment->getCenterx(g_invmap[s_iPed], s_iMes);}
	/**
	 * Get the Y position coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @param s_iMes index of the frame
	 * @return Y position coordinate
	 */
	inline float getExpPositionySimulated(int s_iPed, int s_iMes)
		{return g_experiment->getCentery(g_invmap[s_iPed], s_iMes);}
	/**
	 * Get the X velocity coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @param s_iMes index of the frame
	 * @return X velocity coordinate
	 */
	inline float getExpVelocityxSimulated(int s_iPed, int s_iMes)
		{return g_experiment->getCenterVelocityx(g_invmap[s_iPed], s_iMes);}
	/**
	 * Get theY velocity coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at a given frame.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @param s_iMes index of the frame
	 * @return Y velocity coordinate
	 */
	inline float getExpVelocityySimulated(int s_iPed, int s_iMes)
		{return g_experiment->getCenterVelocityy(g_invmap[s_iPed], s_iMes);}
	
	/**
	 * Get the X position coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @return X position coordinate
	 */
	inline float getExpPositionxSimulated(int s_iPed)
		{return g_experiment->getCenterx(g_invmap[s_iPed]);}
	/**
	 * Get the Y position coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @return Y position coordinate
	 */
	inline float getExpPositionySimulated(int s_iPed)
		{return g_experiment->getCentery(g_invmap[s_iPed]);}
	/**
	 * Get the X velocity coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @return X velocity coordinate
	 */
	inline float getExpVelocityxSimulated(int s_iPed)
		{return g_experiment->getCenterVelocityx(g_invmap[s_iPed]);}
	/**
	 * Get theY velocity coordinate of an agent (simulated) from @ref Record::g_experiment
	 * at the current time.
	 * @param s_iPed index of the agent, uses @ref Record::g_invmap
	 * @return Y velocity coordinate
	 */
	inline float getExpVelocityySimulated(int s_iPed)
		{return g_experiment->getCenterVelocityy(g_invmap[s_iPed]);}
	
	/**
	 * Get the X position coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return X position coordinate
	 */
	inline float getSimPositionxSimulated(int s_iPed, int s_iMes)
		{return g_simulation->getCenterx(s_iPed, s_iMes);}
	/**
	 * Get the Y position coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return Y position coordinate
	 */
	inline float getSimPositionySimulated(int s_iPed, int s_iMes)
		{return g_simulation->getCentery(s_iPed, s_iMes);}
	/**
	 * Get the X velocity coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return X velocity coordinate
	 */
	inline float getSimVelocityxSimulated(int s_iPed, int s_iMes)
		{return g_simulation->getCenterVelocityx(s_iPed, s_iMes);}
	/**
	 * Get the Y velocity coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at a given frame.
	 * @param s_iPed index of the agent
	 * @param s_iMes index of the frame
	 * @return Y velocity coordinate
	 */
	inline float getSimVelocityySimulated(int s_iPed, int s_iMes)
		{return g_simulation->getCenterVelocityy(s_iPed, s_iMes);}
	
	/**
	 * Get the X position coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at the current time.
	 * @param s_iPed index of the agent
	 * @return X position coordinate
	 */
	inline float getSimPositionxSimulated(int s_iPed)
		{return g_simulation->getCenterx(s_iPed);}
	/**
	 * Get the Y position coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at the current time.
	 * @param s_iPed index of the agent
	 * @return Y position coordinate
	 */
	inline float getSimPositionySimulated(int s_iPed)
		{return g_simulation->getCentery(s_iPed);}
	/**
	 * Get the X velocity coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at the current time.
	 * @param s_iPed index of the agent
	 * @return X velocity coordinate
	 */
	inline float getSimVelocityxSimulated(int s_iPed)
		{return g_simulation->getCenterVelocityx(s_iPed);}
	/**
	 * Get the Y velocity coordinate of an agent (simulated) from @ref Record::g_simulation
	 * at the current time.
	 * @param s_iPed index of the agent
	 * @return Y velocity coordinate
	 */
	inline float getSimVelocityySimulated(int s_iPed)
		{return g_simulation->getCenterVelocityy(s_iPed);}
	
	/**
	 * Get the next X position coordinate of an agent (simulated) from @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @return next X position coordinate
	 */
	inline float getSimPositionxNextSimulated(int s_iPed)
		{return g_simulation->getCenterxNext(s_iPed);}
	/**
	 * Get the next Y position coordinate of an agent (simulated) from @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @return next Y position coordinate
	 */
	inline float getSimPositionyNextSimulated(int s_iPed)
		{return g_simulation->getCenteryNext(s_iPed);}
	/**
	 * Get the next X velocity coordinate of an agent (simulated) from @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @return next X velocity coordinate
	 */
	inline float getSimVelocityxNextSimulated(int s_iPed)
		{return g_simulation->getCenterVelocityxNext(s_iPed);}
	/**
	 * Get the next Y velocity coordinate of an agent (simulated) from @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @return next Y velocity coordinate
	 */
	inline float getSimVelocityyNextSimulated(int s_iPed)
		{return g_simulation->getCenterVelocityyNext(s_iPed);}
	
	/**
	 * Set the position of an agent in @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_x X position coordinate
	 * @param s_y Y position coordinate
	 */
	inline void setSimPosition(int s_iPed, float s_x, float s_y)
		{g_simulation->setPosition(g_map[s_iPed], s_x, s_y);}
	/**
	 * Set the velocity of an agent in @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_x X velocity coordinate
	 * @param s_y Y velocity coordinate
	 */
	inline void setSimVelocity(int s_iPed, float s_x, float s_y)
		{g_simulation->setVelocity(g_map[s_iPed], s_x, s_y);}
	/**
	 * Set the goal of an agent in @ref Record::g_simulation.
	 * @param s_iPed index of the agent, uses @ref Record::g_map
	 * @param s_x X goal coordinate
	 * @param s_y Y goal coordinate
	 */
	inline void setSimGoal(int s_iPed, float s_x, float s_y)
		{g_simulation->setGoal(g_map[s_iPed], s_x, s_y);}
	
	/**
	 * Set the position of an agent (simulated) in @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @param s_x X position coordinate
	 * @param s_y Y position coordinate
	 */
	inline void setSimPositionSimulated(int s_iPed, float s_x, float s_y)
		{g_simulation->setPosition(s_iPed, s_x, s_y);}
	/**
	 * Set the velocity of an agent (simulated) in @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @param s_x X velocity coordinate
	 * @param s_y Y velocity coordinate
	 */
	inline void setSimVelocitySimulated(int s_iPed, float s_x, float s_y)
		{g_simulation->setVelocity(s_iPed, s_x, s_y);}
	/**
	 * Set the goal of an agent (simulated) in @ref Record::g_simulation.
	 * @param s_iPed index of the agent
	 * @param s_x X goal coordinate
	 * @param s_y Y goal coordinate
	 */
	inline void setSimGoalSimulated(int s_iPed, float s_x, float s_y)
		{g_simulation->setGoal(s_iPed, s_x, s_y);}
	
	/**
	 * Sets @ref Record::g_experiment.
	 * @param s_experiment the new @ref Experiment object
	 */
	inline void setExperiment(Experiment * s_experiment)
        {g_experiment = s_experiment;
        g_nMeasure = g_experiment->getNMeasure();};
	/**
	 * Sets @ref Record::g_simulation.
	 * @param s_simulation the new @ref Simulation object
	 */
	inline void setSimulation(Simulation * s_simulation)
		{g_simulation = s_simulation; g_params = g_simulation->getParam(0, 0); g_nParams = g_simulation->getNParam();};
	
	/** Returns @ref Record::g_experiment. */
	inline Experiment * getExperiment()
		{return g_experiment;};
	/** Returns @ref Record::g_simulation. */
	inline Simulation * getSimulation()
		{return g_simulation;};
};

}

#endif /* CRAAL_RECORD_H_ */
