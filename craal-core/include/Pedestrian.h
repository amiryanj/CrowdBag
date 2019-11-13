#ifndef CRAAL_PEDESTRIAN_H_
#define CRAAL_PEDESTRIAN_H_

#include "CraalIncludes.h"
#include "SimulationGeneric.h"
#include "PDF.h"

namespace craal {

/**
 * The base Pedestrian class which should not be used; serves when a simulation model is only implemented as "an agent class".
 */
class Pedestrian
{
protected:
	float g_distToGoal;
	float * g_params;
	int g_nbParams;

public:
	static std::vector<std::string> * g_names;
	static std::vector<PDF> * g_pdfs;
	static std::vector<float> * g_defaults;
	std::string name;
	
	Pedestrian() : g_distToGoal(0), g_params(0), g_nbParams(g_defaults->size())
	{
		g_params = new float[g_nbParams];
	};
	virtual ~Pedestrian() {if (g_params) delete [] g_params;};
	
	virtual void reset() = 0;
	virtual void addPedestrianInteraction(Pedestrian * p) = 0;
	virtual void addObstacle(float s_startx, float s_starty, float s_endx, float s_endy) = 0;
	
	virtual void setPosition(float s_x, float s_y) = 0;
	virtual void setVelocity(float s_x, float s_y) = 0;
	virtual void setGoal(float s_x, float s_y) = 0;
	
	virtual void updateVelocity(float s_dt) = 0;
	virtual void updatePosition(float s_dt) = 0;
	
	virtual float getCenterx() = 0;
	virtual float getCentery() = 0;
	virtual float getCenterVelocityx() = 0;
	virtual float getCenterVelocityy() = 0;
	
	static void addParam(std::string s_name, float s_default, PDF s_pdf)
		{g_defaults->push_back(s_default); g_pdfs->push_back(s_pdf); g_names->push_back(s_name); /*addConfigurable(s_name, new Type_pdf, &(g_pdfs->back));*/};
	
	inline int getNParams(){return g_nbParams;};
	inline float getDistanceToGoal() {return g_distToGoal;};
	inline float * getParam(int s_indParam) {return g_params+s_indParam;};
};

}

#endif /* CRAAL_PEDESTRIAN_H_ */
