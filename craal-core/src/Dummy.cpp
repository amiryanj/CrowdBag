#include "../include/Dummy.h"

namespace craal {

void Dummy::updateVelocity(float s_dt)
{
	g_vx = g_gx-g_x;
	g_vy = g_gy-g_y;
	
	float tmp = g_vx*g_vx+g_vy*g_vy;
	
	if (tmp > g_speed*g_speed) {
		tmp = g_speed/sqrt(tmp);
		g_vx *= tmp;
		g_vy *= tmp;
	}
}

void Dummy::updatePosition(float s_dt)
{
	g_x += g_vx*s_dt;
	g_y += g_vy*s_dt;
	
	float dx, dy;
	
	dx = g_gx-g_x;
	dy = g_gy-g_y;
	
	g_distToGoal = sqrt(dx*dx+dy*dy);
}

}
