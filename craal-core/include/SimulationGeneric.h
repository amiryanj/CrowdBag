#ifndef CRAAL_SIMULATIONGENERIC_H_
#define CRAAL_SIMULATIONGENERIC_H_

#include "CraalIncludes.h"

#include "Pedestrian.h"
#include "Simulation.h"

namespace craal {

/**
 * Generic model implementation of @ref Simulation, used for models implemented
 * as individual agents (see @ref Pedestrian).
 */
template <class T>
class SimulationGeneric : public Simulation
{
private:
	T * g_pedestrians;
	std::vector<std::string> g_names;
	
	void p_delete2();

public:
	SimulationGeneric();
	virtual ~SimulationGeneric();
	
	virtual void init();
	virtual void reset();
	
	virtual void addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy);
	
	virtual void setPosition(int s_indPedestrian, float s_x, float s_y);
	virtual void setVelocity(int s_indPedestrian, float s_x, float s_y);
	virtual void setGoal(int s_indPedestrian, float s_x, float s_y);
	
	virtual void doStep(float s_dt = 0.01);
};

template <class T>
SimulationGeneric<T>::SimulationGeneric() :
	g_pedestrians(0)
{
	T::g_defaults = &g_defaults;
	T::g_pdfs = &g_pdfs;
	T::g_names = &g_names;
	T::setParams();
	
	g_nbParams = g_defaults.size();
	
	for (int i = 0; i < g_nbParams; i++)
		addConfigurable(g_names[i], new Type_pdf, &(g_pdfs[i]));
	
	T obj;
	name = obj.name;
}

template <class T>
SimulationGeneric<T>::~SimulationGeneric()
{
	p_delete2();
}

template <class T>
void SimulationGeneric<T>::p_delete2()
{
	if (g_pedestrians) delete [] g_pedestrians;
	g_pedestrians = 0;
}

template <class T>
void SimulationGeneric<T>::init()
{
	p_delete2();
	g_pedestrians = new T[g_nPedestrian];
	
	for (int i = 0; i < g_nPedestrian; i++) {
		for (int j = 0; j < g_nPedestrian; j++) {
			if (i != j) {
				g_pedestrians[i].addPedestrianInteraction(g_pedestrians + j);
			}
		}
	}
	
	setNParams(g_defaults.size());
}

template <class T>
void SimulationGeneric<T>::reset()
{
	for (int i = 0; i < g_nPedestrian; i++) {
		g_pedestrians[i].reset();
		
		memcpy(g_pedestrians[i].getParam(0), g_params+i*g_nbParams, g_nbParams*sizeof(float));
	}
}

template <class T>
void SimulationGeneric<T>::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{
	for (int i = 0; i < g_nPedestrian; i++) {
		g_pedestrians[i].addObstacle(s_startx, s_starty, s_endx, s_endy);
	}
}

template <class T>
void SimulationGeneric<T>::setPosition(int s_indPedestrian, float s_x, float s_y)
{
	g_pedestrians[s_indPedestrian].setPosition(s_x, s_y);
}

template <class T>
void SimulationGeneric<T>::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
	g_pedestrians[s_indPedestrian].setVelocity(s_x, s_y);
}

template <class T>
void SimulationGeneric<T>::setGoal(int s_indPedestrian, float s_x, float s_y)
{
	g_pedestrians[s_indPedestrian].setGoal(s_x, s_y);
}

template <class T>
void SimulationGeneric<T>::doStep(float s_dt)
{
	for (int i = 0; i < g_nPedestrian; i++) {
		g_pedestrians[i].updateVelocity(s_dt);
	}
	for (int i = 0; i < g_nPedestrian; i++) {
		g_pedestrians[i].updatePosition(s_dt);
		
		setNextState(i,
					 g_pedestrians[i].getCenterx(),
					 g_pedestrians[i].getCentery(),
					 g_pedestrians[i].getCenterVelocityx(),
					 g_pedestrians[i].getCenterVelocityy());
	}
}

}

#endif /* CRAAL_SIMULATIONGENERIC_H_ */
