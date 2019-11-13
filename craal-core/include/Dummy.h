#ifndef CRAAL_DUMMY_H_
#define CRAAL_DUMMY_H_

#include "Pedestrian.h"

namespace craal {

/**
 * Dummy model implementation of @ref Simulation, agents go straight to their goals, no collision avoidance.
 */
class Dummy : public Pedestrian
{
private:
	float g_x, g_y;
	float g_vx, g_vy;
	float g_gx, g_gy;
	
	float g_speed;
	
public:
	Dummy() : g_speed(1.4){};
	virtual ~Dummy(){};
	
	virtual void reset(){};
	virtual void addPedestrianInteraction(Pedestrian * p){};
	virtual void addObstacle(float s_startx, float s_starty, float s_endx, float s_endy){};
	
	virtual void setPosition(float s_x, float s_y)
		{g_x = s_x, g_y = s_y;};
	virtual void setVelocity(float s_x, float s_y)
		{g_vx = s_x, g_vy = s_y;};
	virtual void setGoal(float s_x, float s_y)
		{g_gx = s_x, g_gy = s_y;};
	
	virtual void updateVelocity(float s_dt);
	virtual void updatePosition(float s_dt);
	
	virtual float getCenterx() {return g_x;};
	virtual float getCentery() {return g_y;};
	virtual float getCenterVelocityx() {return g_vx;};
	virtual float getCenterVelocityy() {return g_vy;};
	
	static void setParams(){};
};

}

#endif /* CRAAL_DUMMY_H_ */
