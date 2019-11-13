#ifndef HELBING_HELBING_H_
#define HELBING_HELBING_H_

#include <stdio.h>
#include "vec2f.h"
#include "intelkdtree.h"

namespace helbing {

const float SOCIAL_SCALING = 2000.f; // 2000 N
const float AGENT_RADIUS_MAX = 0.35f; // 0.7m diamater
const float AGENT_REACTION_TIME = 0.5f; // 0.5s
const float REPULSE_SPRING_CONSTANT = 120000.f; // 2.4e4 kg/sec^2
const float COEFF_SLIDING_FRICTION = 240000.f;

void setNearObstacles(Agent& a);
vec2f closestPointLine2(vec2f Point, vec2f LineStart, vec2f LineDir, double& dist);

class HelbingSim{
public:
	float timeStep;
	int numAgents;
	KDTreeSlidingMidpoint *kdtree;
	bool doBoids;

	HelbingSim(){
		numAgents = 0;
        numObstacles = 0;
		doBoids = false;
	}

	~HelbingSim(){
		clearSim();
	}

	void setTimeStep(float ts){
		timeStep = ts;
	}

	void init(){
		   //TODO: Remove KD TREE!!!
		  kdtree = new KDTreeSlidingMidpoint(2*numAgents,numAgents);
	}
	void clearSim(){
		delete kdtree;
	}

	Agent getAgent(int id){
		return agentlist[id];
	}

	void setRadius(int id, float r){
		agentlist[id].radius = r;
	}

	void setGoalVel(int id, float gvx, float gvy){
		agentlist[id].goalVel.x = gvx;
		agentlist[id].goalVel.y = gvy;
	}

	void setPos(int id, float px, float py){
		agentlist[id].pos.x = px;
		agentlist[id].pos.y = py;
	}

	void setVel(int id, float vx, float vy){
		agentlist[id].vel.x = vx;
		agentlist[id].vel.y = vy;
	}

	void addAgent(float px, float py, float vx, float vy){
		Agent a;
		a.ID = numAgents++;
		a.colliding = false;
		a.pos.x = px;
		a.pos.y = py;
		a.vel.x = vx;
		a.vel.y = vy;
		a.sight = 10;
		for (int i = 0; i < MAX_NEAR_OBS; i++){
			a.nearObstacle[i] = NULL;
		}
		for (int i = 0; i < MAX_NEIGHBORS; i++){
			a.neighbors[i] = NULL;
		}
		agentlist[a.ID] = a;
	}

	void addObstacle(float ax, float ay, float bx, float by){
		lineObstacle l;
		l.a = vec2f(ax,ay);
		l.b = vec2f(bx,by);
		obstacles[numObstacles] = l;
		numObstacles++;
	}

	void computeNeighbors(Agent* a){
		kdtree->setNeighbors(*a);
	}

	void doStep(){
		kdtree->generateKDTreeSlidingMidpoint(agentlist,numAgents);
		for (int i = 0; i < numAgents; i++){
			computeNeighbors(&agentlist[i]);
			setNearObstacles(agentlist[i]);
		}
		for (int i = 0; i < numAgents; i++){
			update(&agentlist[i]);
		}
		for (int i = 0; i < numAgents; i++){
			agentlist[i].vel = agentlist[i].nextVel;
			agentlist[i].pos = agentlist[i].pos+agentlist[i].vel*timeStep;
		}
	}

	void update(Agent* a){
		vec2f newvel;
		if (doBoids){
			newvel = computeBoids(a->goalVel,*a,a->neighbors,a->colliding);
		} else {
			newvel = computeHelbing(a->goalVel,*a,a->neighbors,a->colliding);
		}
		float mag = newvel.magnitude();
		if (mag > 2.5){
			newvel = newvel / mag;
			newvel = newvel * 2.5;
		}
		a->nextVel.x = newvel.x;
		a->nextVel.y = newvel.y;
	}

	vec2f computeBoids(vec2f veli, Agent& a, Agent* b[], bool colliding){
        int i = 0, n = 0;
        float maxCollideDist = 5.4*a.radius;
        vec2f force = vec2f(0,0);
        vec2f toGoal = (a.goalVel - a.vel);
        vec2f noCollide = vec2f(0,0);
        double dist = 0;
        if (b[0] != NULL){
            dist = a.pos.distanceto(b[0]->pos);
            if (dist < maxCollideDist)
                noCollide = noCollide + (a.pos-b[0]->pos);
        }
        if (a.nearObstacle[0] != NULL){
            vec2f nearestPt = closestPointLine2(a.pos,a.nearObstacle[0]->a,a.nearObstacle[0]->b-a.nearObstacle[0]->a,dist);
            if (dist < maxCollideDist)
                noCollide = noCollide + ( a.pos-nearestPt)*7.6;
        }

        float scale = 10* pow((maxCollideDist-dist)/maxCollideDist,2);

        vec2f acc = toGoal * .35 + noCollide*scale * .65;

        vec2f vel = a.vel + acc*timeStep*10;
        //vel.x = -2; vel.y = -2;
        return vel;
	}

	vec2f computeHelbing(vec2f veli, Agent& a, Agent* b[], bool colliding){
		int i = 0, n = 0;
		vec2f force = vec2f(0,0);
		float mass = 80;
		force = force + (veli - a.vel) * mass / AGENT_REACTION_TIME;

		for (; n < MAX_NEIGHBORS; i++,n++){
			if (b[i] == NULL) break;
			vec2f normal_ij = a.pos - b[i]->pos;
			float distance_ij = normal_ij.magnitude();
			float Radii_ij = a.radius + b[i]->radius; 
			vec2f f_social = normal_ij * (SOCIAL_SCALING * expf((Radii_ij - distance_ij)/.08));

			vec2f f_pushing = vec2f(0.f, 0.f); 
			vec2f f_friction = vec2f(0.f, 0.f);

			//if (distance_ij > 2) continue;

			if (distance_ij < Radii_ij) {
				// pushing
				vec2f tangent_ij( normal_ij.y, -normal_ij.x); 

				// make sure direction is opposite agent velocity
                if (dot(tangent_ij, a.vel) < 0.f) {
					tangent_ij = tangent_ij * -1; 
				}
				f_pushing = normal_ij * (REPULSE_SPRING_CONSTANT * (Radii_ij - distance_ij)); 

                f_friction = tangent_ij * (COEFF_SLIDING_FRICTION * (Radii_ij - distance_ij)) * (dot((b[i]->vel - a.vel) , tangent_ij) /distance_ij);
			}

			force = force + f_social + f_pushing + f_friction; 

			//b[i]->pos
        }

		for (int obs = 0; obs < MAX_NEAR_OBS; obs++){
		if (a.nearObstacle[obs] == NULL) break;
		double dist;
		vec2f nearestPt = closestPointLine2(a.pos,a.nearObstacle[obs]->a,a.nearObstacle[obs]->b-a.nearObstacle[obs]->a,dist);
		vec2f normal_io = a.pos - nearestPt;
		float dist_io = normal_io.magnitude(); 
		normal_io = normal_io/dist_io; 

		vec2f f_obs = normal_io * (SOCIAL_SCALING * exp((a.radius - dist_io) / 0.08f)); 
		vec2f f_pushing = vec2f(0.f, 0.f);
		vec2f f_friction = vec2f(0.f, 0.f);

		// pushing, friction
		if (dist_io < a.radius) { // intersection has occurred
		vec2f tangent_io( normal_io.y, -normal_io.x); 

		// make sure direction is opposite i's velocity
		if (dot(tangent_io,a.vel) < 0.f) {
		tangent_io = tangent_io *-1;
		}

		f_pushing = normal_io * (REPULSE_SPRING_CONSTANT * (a.radius  - dist_io)); 

		// friction
		f_friction = tangent_io * COEFF_SLIDING_FRICTION * (a.radius - dist_io) * dot(a.vel,tangent_io);
		}
		force = force +  f_obs + f_pushing - f_friction; 
		}
		
		vec2f acc = force / mass;
		vec2f vel = a.vel + acc*timeStep;
		return vel;
	}

};

}

#endif /* HELBING_HELBING_H_ */
