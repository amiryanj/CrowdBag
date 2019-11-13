#ifndef HELBING3D_INTELKDTREE_H_
#define HELBING3D_INTELKDTREE_H_

#include <float.h>
#include <stdio.h>
#include <stdlib.h>

namespace helbing3d {

//#include "obstacle.h"
#define MAX_AGENTS 1024
#define MAX_NEAR_OBS 10
const int MAX_NEIGHBORS = 300;
class Agent{
public:
	unsigned ID;
	vec3f pos;
	vec3f vel;
	float radius;
	vec3f goalVel;
	vec3f nextVel;
	bool colliding;
	float sight;
	Agent* neighbors[MAX_NEIGHBORS];
	//lineObstacle* nearObstacle[MAX_NEAR_OBS];
};
extern Agent agentlist[MAX_AGENTS];
#define sqare(X) (X*X)


#define EPSILOM 1e-3

#define MAX_AGENTS_PER_LEAF 300
void swap(int &x, int &y);

int partition(int * agents, int agentstart, int agentend,
	      int split_axis, float pivotval,
	      float * agentxcoord, float * agentycoord, float * agentzcoord,
	      int & minagent,int & maxagent);

struct KDTreeSlidingMidpointNode{
  int split_axis;
  float split_point;

  int numagents;
  int agentstart, agentend;

  int left_child, right_child;
  float bbminX, bbminY, bbmaxX, bbmaxY, bbminZ, bbmaxZ; // bounding box
  bool leaf_node;

  // for statistics
  int numvisited;

  KDTreeSlidingMidpointNode(){
  }
};


struct treeParamStruct {
  int realID;
  int node;
  float * agentxcoord; float * agentycoord; float * agentzcoord;
  unsigned numAgents; int depth;
};

struct KDTreeSlidingMidpoint{
  KDTreeSlidingMidpointNode * tree;
  treeParamStruct * treeParams;
  int * agents;
  float * agentxcoord, * agentycoord , * agentzcoord;

  unsigned numnodes;
  unsigned maxnodes;

  int maxdepth;

#ifdef USE_SDL
  void draw(){
    //for (int i = 0; i < numnodes; i++){
    //    tree[i].draw();
    //}
    drawRecursive(tree[0]);
  }

  void drawRecursive(KDTreeSlidingMidpointNode& node){
    if (!node.leaf_node){
        drawRecursive(tree[node.left_child]);
        drawRecursive(tree[node.right_child]);
    }
    else{
        glColor3f(0,.1,.2);
        glLineWidth(2);
        glBegin(GL_LINE_LOOP);
            glVertex3f(node.bbminX,node.bbminY,.5);  // ???
            glVertex3f(node.bbminX,node.bbmaxY,.5);
            glVertex3f(node.bbmaxX,node.bbmaxY,.5);
            glVertex3f(node.bbmaxX,node.bbminY,.5);
        glEnd();
    }
  }
#endif

  KDTreeSlidingMidpoint(){ tree = NULL; agents = NULL; numnodes = 0; }
  KDTreeSlidingMidpoint(unsigned maxnodes_, unsigned numAgents){
    //printf("Calling constructor\n"); fflush(stdout);
    maxnodes = maxnodes_;
    tree = (KDTreeSlidingMidpointNode *)malloc(sizeof(KDTreeSlidingMidpointNode) * maxnodes);
#ifdef PARALLEL_KDCons
    treeParams = (treeParamStruct *)malloc(sizeof(treeParamStruct) * nthreads * 32);
#endif
    agents = (int *)malloc(sizeof(int) * numAgents);
    agentxcoord = (float *)malloc(sizeof(float) * numAgents);
    agentycoord = (float *)malloc(sizeof(float) * numAgents);
	agentzcoord = (float *)malloc(sizeof(float) * numAgents);

    for(unsigned i = 0; i < maxnodes; i++) {
      tree[i].left_child = tree[i].right_child = -1;
      tree[i].numagents = 0;
      tree[i].bbminX = FLT_MAX; tree[i].bbminY = FLT_MAX; tree[i].bbminZ = FLT_MAX; tree[i].bbmaxX = -FLT_MAX; tree[i].bbmaxY = -FLT_MAX; tree[i].bbmaxZ = -FLT_MAX;

      tree[i].agentstart = tree[i].agentend = -1;

      tree[i].leaf_node = 0;
      tree[i].split_axis = -1; tree[i].split_point = -FLT_MAX;

      tree[i].numvisited = 0;
    }
    numnodes = 0; maxdepth = 0;
  }

  ~KDTreeSlidingMidpoint(){
    //printf("Calling destructor\n"); fflush(stdout);
	free(agentxcoord);
    free(agentycoord);
	free(agentzcoord);
    free(agents);
    free(tree);
  }

  void generateKDTreeSlidingMidpoint(Agent agentlist[], unsigned numAgents);

/*   void generateKDTreeSlidingMidpoint(float * agentxcoord, float * agentycoord,  */
/* 				     unsigned numAgents); */
  void generateKDTreeSlidingMidpointRecursive(int node, int readID, float * agentxcoord, float * agentycoord, float * agentzcoord,
				     unsigned numAgents, int depth, int threadid);
#ifdef PARALLEL_KDCons
  static void generateKDTreeSlidingMidpointRecursiveParallel(long threadid, void * arg);
#endif
  void print();
  void printRecursive(int node, int depth);

  void setNeighbors(Agent& a);
  //  void setNeighbors(float * agentxcoord, float * agentycoord, unsigned int agentindex, float radius, int k, unsigned int * outputagents, float * outputagentsdist, unsigned & numoutputs);
  void setNeighborsRecursive(float * agentxcoord, float * agentycoord, float * agentzcoord, unsigned int agentindex, float radius, int k, Agent * outputagents[], float * outputagentsdist, unsigned & numoutputs, int node, float & mindist);

  //  void setNeighborsRecursive(float * agentxcoord, float * agentycoord, unsigned int agentindex, float radius, int k,  unsigned & numoutputs, int node, float & mindist);

};

extern KDTreeSlidingMidpoint *kdtree;
#ifdef PARALLEL_KDCons
 void KDTreeSlidingMidpoint::generateKDTreeSlidingMidpointRecursiveParallel(long threadid, void * arg){
   treeParamStruct* tp = (treeParamStruct*)arg;
//printf("!RealID=%d (%d)\n",tp->realID,threadid); fflush(stdout);
    kdtree->generateKDTreeSlidingMidpointRecursive(tp->node,tp->realID,tp->agentxcoord,tp->agentycoord,tp->agentzcoord,tp->numAgents,tp->depth,threadid);
}
#endif

}

#endif /* HELBING3D_INTELKDTREE_H_ */
