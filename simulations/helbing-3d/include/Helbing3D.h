#ifndef HELBING3D_HELBING_H_
#define HELBING3D_HELBING_H_

#include <stdio.h>
#include "vec3f.h"
#include "intelkdtree.h"

namespace helbing3d {

const float SOCIAL_SCALING = 2000.f; // 2000 N
const float AGENT_RADIUS_MAX = 0.35f; // 0.7m diamater
const float AGENT_REACTION_TIME = 0.5f; // 0.5s
const float REPULSE_SPRING_CONSTANT = 120000.f; // 2.4e4 kg/sec^2
const float COEFF_SLIDING_FRICTION = 240000.f;

void setNearObstacles(Agent& a);
vec3f closestPointLine2(vec3f Point, vec3f LineStart, vec3f LineDir, double& dist);

class HelbingSim{
public:
	float timeStep;
	int numAgents;
	KDTreeSlidingMidpoint *kdtree;
	bool doBoids;

	HelbingSim(){
		numAgents = 0;
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

	void setGoalVel(int id, float gvx, float gvy, float gvz){
		agentlist[id].goalVel.x = gvx;
		agentlist[id].goalVel.y = gvy;
		agentlist[id].goalVel.z = gvz;
	}

	void setPos(int id, float px, float py, float pz){
		agentlist[id].pos.x = px;
		agentlist[id].pos.y = py;
		agentlist[id].pos.z = pz;
	}

	void setVel(int id, float vx, float vy, float vz){
		agentlist[id].vel.x = vx;
		agentlist[id].vel.y = vy;
		agentlist[id].vel.z = vz;
	}

	void addAgent(float px, float py, float pz, float vx, float vy, float vz){
		Agent a;
		a.ID = numAgents++;
		a.colliding = false;
		a.pos.x = px;
		a.pos.y = py;
		a.pos.z = pz;
		a.vel.x = vx;
		a.vel.y = vy;
		a.vel.z = vz;
		a.sight = 10;
		for (int i = 0; i < MAX_NEIGHBORS; i++){
			a.neighbors[i] = NULL;
		}
		agentlist[a.ID] = a;
	}

	// Insect 2014
	//void addObstacle(float ax, float ay, float bx, float by){
	//	lineObstacle l;
	//	l.a = vec2f(ax,ay);
	//	l.b = vec2f(bx,by);
	//	obstacles[numObstacles] = l;
	//	numObstacles++;
	//}

	void computeNeighbors(Agent* a){
		kdtree->setNeighbors(*a);
	}

	void doStep(){
		kdtree->generateKDTreeSlidingMidpoint(agentlist,numAgents);
		for (int i = 0; i < numAgents; i++){
			computeNeighbors(&agentlist[i]);
// 			setNearObstacles(agentlist[i]);
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
		vec3f newvel;
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
		a->nextVel.z = newvel.z;
	}

	vec3f computeBoids(vec3f veli, Agent& a, Agent* b[], bool colliding){
        int i = 0, n = 0;
        float maxCollideDist = 5.4*a.radius;
        vec3f force = vec3f(0,0,0);
        vec3f toGoal = (a.goalVel - a.vel);
        vec3f noCollide = vec3f(0,0,0);
        double dist = 0;
        if (b[0] != NULL){
            dist = a.pos.distanceto(b[0]->pos);
            if (dist < maxCollideDist)
                noCollide = noCollide + (a.pos-b[0]->pos);
        }

        //if (a.nearObstacle[0] != NULL){ // ???
        //    vec3f nearestPt = closestPointLine2(a.pos,a.nearObstacle[0]->a,a.nearObstacle[0]->b-a.nearObstacle[0]->a,dist);
        //    if (dist < maxCollideDist)
        //        noCollide = noCollide + ( a.pos-nearestPt)*7.6;
        //}

        float scale = 10* pow((maxCollideDist-dist)/maxCollideDist,2);

        vec3f acc = toGoal * .35 + noCollide*scale * .65;

        vec3f vel = a.vel + acc*timeStep*10;
        //vel.x = -2; vel.y = -2;
        return vel;
	}

	vec3f computeHelbing(vec3f veli, Agent& a, Agent* b[], bool colliding){
		int i = 0, n = 0;
		vec3f force = vec3f(0,0,0);
		float mass = 80;
		force = force + (veli - a.vel) * mass / AGENT_REACTION_TIME;

		for (; n < MAX_NEIGHBORS; i++,n++){
			if (b[i] == NULL) break;
			vec3f normal_ij = a.pos - b[i]->pos;
			float distance_ij = normal_ij.magnitude();
			float Radii_ij = a.radius + b[i]->radius; 
			vec3f f_social = normal_ij * (SOCIAL_SCALING * expf((Radii_ij - distance_ij)/.08));

			vec3f f_pushing = vec3f(0.f, 0.f, 0.f); 
			vec3f f_friction = vec3f(0.f, 0.f, 0.f);

			if (distance_ij < Radii_ij) {
				// pushing
				//vec2f tangent_ij( normal_ij.y, -normal_ij.x); 

				vec3f tangent_ij = cross(normalize(cross(normal_ij, a.vel)), normal_ij);

				// make sure direction is opposite agent velocity
                if (dot(tangent_ij, a.vel) < 0.f) {
					tangent_ij = tangent_ij * -1; 
				}
				f_pushing = normal_ij * (REPULSE_SPRING_CONSTANT * (Radii_ij - distance_ij)); 

                f_friction = tangent_ij * (COEFF_SLIDING_FRICTION * (Radii_ij - distance_ij)) * (dot((b[i]->vel - a.vel) , tangent_ij) /distance_ij);
			}

			force = force + f_social + f_pushing + f_friction; 
		}

		//for (int obs = 0; obs < MAX_NEAR_OBS; obs++){
		//	if (a.nearObstacle[obs] == NULL) break;
		//	double dist;
		//	vec2f nearestPt = closestPointLine2(a.pos,a.nearObstacle[obs]->a,a.nearObstacle[obs]->b-a.nearObstacle[obs]->a,dist);
		//	vec2f normal_io = a.pos - nearestPt;
		//	float dist_io = normal_io.magnitude(); 
		//	normal_io = normal_io/dist_io; 

		//	vec2f f_obs = normal_io * (SOCIAL_SCALING * exp((a.radius - dist_io) / 0.08f)); 
		//	vec2f f_pushing = vec3f(0.f, 0.f, 0.f);
		//	vec2f f_friction = vec3f(0.f, 0.f, 0.f);

		//	// pushing, friction
		//	if (dist_io < a.radius) { // intersection has occurred
		//		vec2f tangent_io( normal_io.y, -normal_io.x); 

		//		// make sure direction is opposite i's velocity
		//		if (dot(tangent_io,a.vel) < 0.f) {
		//		tangent_io = tangent_io *-1;
		//		}

		//		f_pushing = normal_io * (REPULSE_SPRING_CONSTANT * (a.radius  - dist_io)); 

		//		// friction
		//		f_friction = tangent_io * COEFF_SLIDING_FRICTION * (a.radius - dist_io) * dot(a.vel,tangent_io);
		//	}
		//	force = force +  f_obs + f_pushing - f_friction; 
		//}
		
		vec3f acc = force / mass;
		vec3f vel = a.vel + acc*timeStep;
		return vel;
	}

};

}

#endif /* HELBING3D_HELBING_H_ */
