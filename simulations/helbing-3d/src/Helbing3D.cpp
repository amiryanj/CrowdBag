#include "../include/Helbing3D.h"

namespace helbing3d {

Agent agentlist[MAX_AGENTS];

int partition(int * agents, int agentstart, int agentend,
	      int split_axis, float pivotval,
	      float * agentxcoord, float * agentycoord, float * agentzcoord,
	      int & minagent, int & maxagent){
  int i = agentstart, j = agentend;
  float minagentcoord = FLT_MAX, maxagentcoord = -FLT_MAX;
  
  // split_axis: 0 - x-axis; 1 - y-axis; 2 - z-axis
  while(1) {

		while((j>=agentstart) && (((split_axis == 0) && (agentxcoord[agents[j]] >= pivotval) ) // near
		   || ( (split_axis == 1) && (agentycoord[agents[j]] >= pivotval)) // right
		   || ( (split_axis == 2) && (agentzcoord[agents[j]] >= pivotval))) // up 
		 ) {
			  // near child
			if( split_axis == 0 ) {
				if(minagentcoord > agentxcoord[agents[j]] ) {
				  minagentcoord = agentxcoord[agents[j]];
				  minagent = j;
				}
			}
			else if( split_axis == 1 ) {
				if(minagentcoord > agentycoord[agents[j]] ) {
				  minagentcoord = agentycoord[agents[j]];
				  minagent = j;
				}
			}
			else if( split_axis == 2) {
				if(minagentcoord > agentzcoord[agents[j]] ) {
				  minagentcoord = agentzcoord[agents[j]];
				  minagent = j;
				}
			}

			j--;
		}

		while((i<=agentend) && ( ( (split_axis == 0) && (agentxcoord[agents[i]] < pivotval) ) //far
		   || ( (split_axis == 1) && (agentycoord[agents[i]] < pivotval)) // left
		   || ( (split_axis == 2) && (agentzcoord[agents[i]] < pivotval)) ) // down
		  ) {
			
			  float agentxcoord_i = agentxcoord[agents[i]], agentycoord_i = agentycoord[agents[i]], agentzcoord_i = agentzcoord[agents[i]];
			  // far child
			  if( split_axis == 0 ) {			
				  if(maxagentcoord < agentxcoord[agents[i]] ) {
				  maxagentcoord = agentxcoord[agents[i]];
				  maxagent = i;
				}
			  }
			  else if( split_axis == 1 ) {			  
				if(maxagentcoord < agentycoord[agents[i]] ) {
				  maxagentcoord = agentycoord[agents[i]];
				  maxagent = i;
				}
			  }
			  else if( split_axis == 2 ) {			  
				if(maxagentcoord < agentzcoord[agents[i]] ) {
				  maxagentcoord = agentzcoord[agents[i]];
				  maxagent = i;
				}
			  }
			  i++;
		}

		if(i < j) swap( agents[i], agents[j]);
		else return j;
  }
}

void KDTreeSlidingMidpoint::generateKDTreeSlidingMidpointRecursive(
       int node, int realID,
       float * agentxcoord, float * agentycoord, float * agentzcoord,
       unsigned numAgents, int depth, int threadid){
		//if (depth <= 3)  printf("RealID=%d\n",realID);
		  
		  if(numAgents <= MAX_AGENTS_PER_LEAF) { 
			  tree[node].leaf_node = 1; 
			  tree[node].split_axis = 0; 
			  return; 
		  }

		  tree[node].leaf_node = 0;


		  // Select split axis to be the longer dimension
		  float tempV1 = tree[node].bbmaxX - tree[node].bbminX;
		  float tempV2 = tree[node].bbmaxY - tree[node].bbminY;
		  float tempV3 = tree[node].bbmaxZ - tree[node].bbminZ;

		  int tempSplit = -1;

		  if( tempV1 < tempV2) {
			  if( tempV1 < tempV3)
				  tempSplit = 0;
			  else
				  tempSplit = 2;
		  }
		  else {
			  if( tempV2 < tempV3)
				  tempSplit = 1;
			  else
				  tempSplit = 2;
		  }

		  // tree[node].split_axis = ( (tree[node].bbmaxX - tree[node].bbminX) < (tree[node].bbmaxY - tree[node].bbminY)); // 0 => split along X, 1 => split along Y
		  tree [node].split_axis = tempSplit;

		  // Choose split point
		  //tree[node].split_point = (tree[node].split_axis)?((tree[node].bbmaxY + tree[node].bbminY)*0.5):((tree[node].bbmaxX + tree[node].bbminX)*0.5); // midpoint
		  if(tree[node].split_axis == 0)
			  tree[node].split_point = (tree[node].bbmaxX + tree[node].bbminX)*0.5;
		  else if(tree[node].split_axis == 1)
			  tree[node].split_point = (tree[node].bbmaxY + tree[node].bbminY)*0.5;
		  else if(tree[node].split_axis == 2)
			  tree[node].split_point = (tree[node].bbmaxZ + tree[node].bbminZ)*0.5;

		  float minagentcoord = FLT_MAX, maxagentcoord = -FLT_MAX;
		  int minagent = -1, maxagent = -1;
		  int minagent_new = -1, maxagent_new = -1;
		  KDTreeSlidingMidpointNode leftC, rightC;

		  if(tree[node].split_axis == 0) { //split along x-axis
			leftC.bbminX =  tree[node].bbminX;
			leftC.bbmaxX =  tree[node].split_point;
			leftC.bbminY =  tree[node].bbminY;
			leftC.bbmaxY =  tree[node].bbmaxY;
			leftC.bbminZ =  tree[node].bbminZ;
			leftC.bbmaxZ =  tree[node].bbmaxZ;

			rightC.bbminX = tree[node].split_point;
			rightC.bbmaxX = tree[node].bbmaxX;
			rightC.bbminY = tree[node].bbminY;
			rightC.bbmaxY = tree[node].bbmaxY;
			rightC.bbminZ = tree[node].bbminZ;
			rightC.bbmaxZ = tree[node].bbmaxZ;
		  }
		  else if (tree[node].split_axis == 1) { //split along y-axis
			leftC.bbminX =  tree[node].bbminX;
			leftC.bbmaxX =  tree[node].bbmaxX;
			leftC.bbminY =  tree[node].bbminY;
			leftC.bbmaxY =  tree[node].split_point;
			leftC.bbminZ =  tree[node].bbminZ;
			leftC.bbmaxZ =  tree[node].bbmaxZ;

			rightC.bbminX = tree[node].bbminX;
			rightC.bbmaxX = tree[node].bbmaxX;
			rightC.bbminY = tree[node].split_point;
			rightC.bbmaxY = tree[node].bbmaxY;
			rightC.bbminZ = tree[node].bbminZ;
			rightC.bbmaxZ = tree[node].bbmaxZ;
		  }
		  else if (tree[node].split_axis == 2) { //split along z-axis
			leftC.bbminX =  tree[node].bbminX;
			leftC.bbmaxX =  tree[node].bbmaxX;
			leftC.bbminY =  tree[node].bbminY;
			leftC.bbmaxY =  tree[node].bbmaxY;
		    leftC.bbminZ =  tree[node].bbminZ;
			leftC.bbmaxZ =  tree[node].split_point;

			rightC.bbminX = tree[node].bbminX;
			rightC.bbmaxX = tree[node].bbmaxX;
			rightC.bbminY = tree[node].bbminY;
			rightC.bbmaxY = tree[node].bbmaxY;
			rightC.bbminZ = tree[node].split_point;
			rightC.bbmaxZ = tree[node].bbmaxZ;
		  }

		  // Find nodes to the left and right of the split_point on the split_axis
		  unsigned partition_index = partition(agents, tree[node].agentstart, tree[node].agentend, tree[node].split_axis, tree[node].split_point, 
			  agentxcoord, agentycoord, agentzcoord, minagent_new, maxagent_new);

		  leftC.agentstart = tree[node].agentstart; leftC.agentend = partition_index;
		  rightC.agentstart = partition_index+1; rightC.agentend = tree[node].agentend;
		  leftC.numagents =
			(leftC.agentend >= 0)?(leftC.agentend - leftC.agentstart + 1):0;
		  rightC.numagents =
			(rightC.agentstart <= tree[node].agentend)?(rightC.agentend - rightC.agentstart + 1):0;

		  // Slide the midpoint
		  if(!leftC.numagents || !rightC.numagents) {
			
			  //float agentcoord = (tree[node].split_axis)?(agentycoord[agents[minagent_new]]):(agentxcoord[agents[minagent_new]]);
			  float agentcoord = -1;
		      if(!leftC.numagents) {
					
				  if(tree[node].split_axis == 0) //split along x-axis
					  agentxcoord[agents[minagent_new]];
				  else if (tree[node].split_axis == 1) //split along y-axis
					  agentycoord[agents[minagent_new]];
				  else if (tree[node].split_axis == 2) //split along z-axis
					  agentzcoord[agents[minagent_new]];

			  
			  tree[node].split_point = agentcoord + EPSILOM;
			  leftC.numagents = 1;
			  {
				 swap(agents[tree[node].agentstart], agents[minagent_new]);
				 leftC.agentstart = leftC.agentend = tree[node].agentstart;
				 rightC.agentstart = tree[node].agentstart + 1; rightC.agentend = tree[node].agentend;
			  }
				 rightC.numagents--;
			}
			else {
			    // tree[node].split_point = ((tree[node].split_axis)?agentycoord[agents[maxagent_new]]:agentxcoord[agents[maxagent_new]]) - EPSILOM;

			   	if(tree[node].split_axis == 0) //split along x-axis
					tree[node].split_point = agentxcoord[agents[maxagent_new]];
				else if (tree[node].split_axis == 1) //split along y-axis
					tree[node].split_point = agentycoord[agents[maxagent_new]];
				else if (tree[node].split_axis == 2) //split along z-axis
					tree[node].split_point = agentzcoord[agents[maxagent_new]];

				tree[node].split_point -= EPSILOM;

			   rightC.numagents = 1;
			   {
					swap(agents[tree[node].agentend], agents[maxagent_new]);
					rightC.agentstart = rightC.agentend = tree[node].agentend;
					leftC.agentstart = tree[node].agentstart; leftC.agentend = tree[node].agentend - 1;
			   }
				 leftC.numagents--;
			}
		
			// Set bounding boxes after sliding
			if(tree[node].split_axis == 0) { //split along x-axis
				leftC.bbmaxX = tree[node].split_point;
				rightC.bbminX = tree[node].split_point;
			}
			else if(tree[node].split_axis == 1) { //split along y-axis
				leftC.bbmaxY = tree[node].split_point;
				rightC.bbminY = tree[node].split_point;
			}
			else if(tree[node].split_axis == 2) { //split along z-axis
				leftC.bbmaxZ = tree[node].split_point;
				rightC.bbminZ = tree[node].split_point;
			}

		  } // End sliding

		  tree[node+1] = leftC;
		  tree[node+2*(leftC.numagents)] = rightC;
		  tree[node].left_child = node+1;
		  tree[node].right_child = node + 2*(leftC.numagents);

		  // recursive call with children
		  numnodes += 2;

#ifdef PARALLEL_KDCons
		if ((1 << depth) == ntasks){
		    // printf("depth %d ntasks %d\n",depth,ntasks);
		    // printf("childIDs %d %d\n",2*realID+1,2*realID+2);
			treeParams[2*realID+1].realID = 2*realID+1;
			treeParams[2*realID+1].node = tree[node].left_child;
			
			treeParams[2*realID+1].agentxcoord = agentxcoord;
			treeParams[2*realID+1].agentycoord = agentycoord;
			treeParams[2*realID+1].agentzcoord = agentzcoord;
			
			treeParams[2*realID+1].numAgents = tree[tree[node].left_child].numagents;
			treeParams[2*realID+1].depth = depth+1;
		    // generateKDTreeSlidingMidpointRecursiveParallel(0,tp);
			
			taskQEnqueueTask1((TaskQTask1)generateKDTreeSlidingMidpointRecursiveParallel,0,(void *)&treeParams[2*realID+1]);
			treeParams[2*realID+2].realID = 2*realID+2;
			treeParams[2*realID+2].node = tree[node].right_child;
			
			treeParams[2*realID+2].agentxcoord = agentxcoord;
			treeParams[2*realID+2].agentycoord = agentycoord;
			treeParams[2*realID+2].agentzcoord = agentzcoord;
			
			treeParams[2*realID+2].numAgents = tree[tree[node].right_child].numagents;
			treeParams[2*realID+2].depth = depth+1;
		
			// generateKDTreeSlidingMidpointRecursiveParallel(0,tp);
			taskQEnqueueTask1((TaskQTask1)generateKDTreeSlidingMidpointRecursiveParallel,0,(void *)&treeParams[2*realID+2]);
		} else
#endif
		{
		    generateKDTreeSlidingMidpointRecursive(tree[node].left_child, 2*realID+1, agentxcoord, agentycoord, agentzcoord, tree[tree[node].left_child].numagents,depth+1,0);
		    generateKDTreeSlidingMidpointRecursive(tree[node].right_child, 2*realID+2, agentxcoord, agentycoord, agentzcoord, tree[tree[node].right_child].numagents,depth+1,0);
		}
}

void KDTreeSlidingMidpoint::generateKDTreeSlidingMidpoint(Agent agentlist[],
							  unsigned numAgents) {
	  
	  if(numAgents <= 0) return;
	  tree[0].numagents = numAgents;
	  tree[0].agentstart = 0; tree[0].agentend = numAgents - 1;
	  tree[0].bbminX = FLT_MAX; tree[0].bbminY = FLT_MAX; tree[0].bbminZ = FLT_MAX;
	  tree[0].bbmaxX = -FLT_MAX; tree[0].bbmaxY = -FLT_MAX; tree[0].bbmaxZ = -FLT_MAX;
	  
	  for(unsigned i = 0; i < numAgents; i++) {
		agentxcoord[i] = agentlist[i].pos.x;
		agentycoord[i] = agentlist[i].pos.y;
		agentycoord[i] = agentlist[i].pos.z;

		agents[i] = i;

		// Set root bounding box
		float agentxcoord_i = agentxcoord[i], agentycoord_i = agentycoord[i], agentzcoord_i = agentzcoord[i];
		if(tree[0].bbminX > agentxcoord_i) tree[0].bbminX = agentxcoord_i;
		if(tree[0].bbmaxX < agentxcoord_i) tree[0].bbmaxX = agentxcoord_i;
		if(tree[0].bbminY > agentycoord_i) tree[0].bbminY = agentycoord_i;
		if(tree[0].bbmaxY < agentycoord_i) tree[0].bbmaxY = agentycoord_i;
		if(tree[0].bbminZ > agentzcoord_i) tree[0].bbminZ = agentzcoord_i;
		if(tree[0].bbmaxZ < agentzcoord_i) tree[0].bbmaxZ = agentzcoord_i;
	  }
	  numnodes = 1;
	  generateKDTreeSlidingMidpointRecursive(0, 0, agentxcoord, agentycoord, agentzcoord, numAgents,0,0);

#ifdef PARALLEL_KDCons
	  taskQWait();
#endif

}


// STILL IN 2D !!!!
void KDTreeSlidingMidpoint::printRecursive(int node, int depth=0){
  printf("Node: %d split_axis: %s BB(%f, %f) - (%f, %f) depth: %d ", node, (tree[node].split_axis==-1)?"INVALID":(tree[node].split_axis?"y":"x"), tree[node].bbminX, tree[node].bbminY, tree[node].bbmaxX, tree[node].bbmaxY, depth);
  if(maxdepth < depth) maxdepth = depth;
  if(!tree[node].leaf_node){
    printf("split_point: %f Agents: ", tree[node].split_point);
    for(unsigned i = tree[node].agentstart; i<= tree[node].agentend; i++){
      printf("%d ", agents[i]);
    }
    printf(" numAgents: %d left_child: %d right_child: %d\n", tree[node].numagents, tree[node].left_child, tree[node].right_child);
    //printf(" numAgents: %d\n", tree[node].numagents);
    printRecursive(tree[node].left_child, depth+1);
    printRecursive(tree[node].right_child, depth+1);
  }else
    printf("\n");

}

// STILL IN 2D !!!!
void KDTreeSlidingMidpoint::print(){
    printRecursive(0, 0);
    printf("maxdepth: %d\n", maxdepth);
    fflush(stdout);
}


void KDTreeSlidingMidpoint::setNeighborsRecursive(float * agentxcoord, float * agentycoord, float * agentzcoord, 
												  unsigned int agentindex, float radius, int k, Agent * outputagents[], float * outputagentsdist, 
												  unsigned & numoutputs, int node, float & mindist){
	int axis = tree[node].split_axis;
	float split = tree[node].split_point;

	//float agentcoord_target = (axis)?agentycoord[agentindex]:agentxcoord[agentindex];
	float agentcoord_target = -1;
	if(axis == 0)
		agentcoord_target = agentxcoord[agentindex];
	else if(axis == 1)
		agentcoord_target = agentycoord[agentindex];
	else if(axis == 2)
		agentcoord_target = agentzcoord[agentindex];

	if(tree[node].leaf_node){

#ifdef STATISTICS
    numleavesexplored++; nnodes++; tree[node].numvisited++;
#endif

    for(unsigned i = tree[node].agentstart; i <= tree[node].agentend; i++){
		if(agents[i] == agentindex) continue;

#ifdef STATISTICS
       ndist++;
#endif

		//float dist = sqrtf((agentxcoord[agents[i]] - agentxcoord[agentindex])* (agentxcoord[agents[i]] - agentxcoord[agentindex]) + (agentycoord[agents[i]] - agentycoord[agentindex])* (agentycoord[agents[i]] - agentycoord[agentindex]))  - a.radius - agentlist[agents[i]].radius;
		float dist = agentlist[agentindex].pos.distanceto(agentlist[agents[i]].pos) - agentlist[agentindex].radius - agentlist[agents[i]].radius;
		if( dist < mindist ) {
			int j;
			for(j = numoutputs - 1; j >= 0 && outputagentsdist[j] > dist; j--)
				if(j < (int)(k - 1)) { outputagentsdist[j + 1] = outputagentsdist[j]; outputagents[j+1] = outputagents[j]; }

				if(j < (int)(k - 1)) { outputagentsdist[j+1] = dist; outputagents[j+1] = &agentlist[agents[i]];}
				if(numoutputs < k)  numoutputs++;
				if(numoutputs == k)  mindist = (outputagentsdist[k - 1]);
		}
	}
    return;
  }

#ifdef STATISTICS
	   nnodes++; tree[node].numvisited++;
#endif

	  bool intersect_left = (split >= (agentcoord_target - mindist - 2*agentlist[0].radius));
	  bool intersect_right = (split < (agentcoord_target + mindist + 2*agentlist[0].radius));
	  bool in_left = (split > agentcoord_target);
	  bool in_right = !(in_left);

/*   if(intersect_left) { */
/*       setNeighborsRecursive(agentxcoord, agentycoord, agentindex, radius, k, outputagents, outputagentsdist, numoutputs, tree[node].left_child, mindist); */
/*       intersect_right = (split < (agentcoord_target + mindist + 2*agentlist[0].radius)); */
/*   } */
/*   if(intersect_right) */
/*       setNeighborsRecursive(agentxcoord, agentycoord, agentindex, radius, k, outputagents, outputagentsdist, numoutputs, tree[node].right_child, mindist); */

	  if(in_left){
		setNeighborsRecursive(agentxcoord, agentycoord, agentzcoord, agentindex, radius, k, outputagents, outputagentsdist, numoutputs, tree[node].left_child, mindist);
		intersect_right = (split < (agentcoord_target + mindist + 2*agentlist[0].radius));
		if(intersect_right)
		  setNeighborsRecursive(agentxcoord, agentycoord, agentzcoord, agentindex, radius, k, outputagents, outputagentsdist, numoutputs, tree[node].right_child, mindist);
	  }
	  else{
		setNeighborsRecursive(agentxcoord, agentycoord, agentzcoord, agentindex, radius, k, outputagents, outputagentsdist, numoutputs, tree[node].right_child, mindist);
		intersect_left = (split >= (agentcoord_target - mindist - 2*agentlist[0].radius));
		if(intersect_left)
		  setNeighborsRecursive(agentxcoord, agentycoord, agentzcoord, agentindex, radius, k, outputagents, outputagentsdist, numoutputs, tree[node].left_child, mindist);
	  }
}


void KDTreeSlidingMidpoint::setNeighbors(Agent &a) {

	float mindist = a.sight;
    a.colliding = false;
    unsigned numnbr = 0;
    float knndist[MAX_NEIGHBORS];
    a.neighbors[MAX_NEIGHBORS-1] = NULL;
    setNeighborsRecursive(agentxcoord, agentycoord, agentzcoord, a.ID, a.sight, MAX_NEIGHBORS, a.neighbors, knndist, numnbr, 0, mindist);
    if (numnbr < MAX_NEIGHBORS) a.neighbors[numnbr] = NULL;
    if (numnbr > 0){
        bool collide = knndist[0] < 0; //a.radius*a.radius;
        a.colliding = collide;
        for(unsigned i = 0; i < numnbr; i++){
          if (collide != (knndist[i] < 0/*a.radius*a.radius*/)){
            a.neighbors[i] = NULL;
            return;
          }
        }
    }
}


void swap(int &x, int &y){
  int tmp= x; x = y; y = tmp;
}


// STILL IN 2D !!!!
//float DistancePointLineSqr(vec2f c, vec2f a, vec2f b) { //Code from Jur van den Berg
//	float r = dot((c - a),(b - a)) / absSq(b - a);
//
//	if (r < 0) { // point a is closest to c
//		return absSq(c - a);
//	} else if (r > 1) { // point b is closest to c
//		return absSq(c - b);
//	} else { // some point in between a and b is closest to c
//		return absSq(c - (a + (b - a)*r));
//	}
//}

// STILL IN 2D !!!!
//void setNearObstacles(Agent& a){
//    a.nearObstacle[0] = NULL;
//    a.nearObstacle[1] = NULL;
//    double dist, minDist2, minDist = 2*a.radius*a.radius;
//    minDist2 = minDist;
//    for (int j = 0; j < numObstacles; j++){
//        //closestPointLine(a.pos,obstacles[j].a,obstacles[j].b,dist);
//        dist = DistancePointLineSqr(a.pos,obstacles[j].a,obstacles[j].b);
//        if (dist < minDist2){
//            if (dist < minDist){
//                minDist2 = minDist;
//                a.nearObstacle[1] = a.nearObstacle[0];
//                minDist = dist;
//                a.nearObstacle[0] = &obstacles[j];
//            }else{
//                minDist2 = dist;
//                a.nearObstacle[1] = &obstacles[j];
//            }
//        }
//    }
//}

// STILL IN 2D !!!!
//vec2f closestPointLine2(vec2f Point, vec2f LineStart, vec2f LineDir, double& dist){
//    double LineMag;
//    double U;
//    vec2f Intersection;
//
//    LineMag = LineDir.magnitude();
//
//    U = ( ( ( Point.x - LineStart.x ) * LineDir.x) +
//        ( (Point.y - LineStart.y) * LineDir.y ) ) /
//        (LineMag * LineMag);
//
//    //printf(" %f %f\n",( Point.x - LineStart.x ) * LineDir.x,(Point.y - LineStart.y) * LineDir.y );
//    //printf("* (%f, %f) : (%f, %f)\n", LineStart.x, LineStart.y, LineDir.x,LineDir.y);
//    if (U < 0.0f) U = 0.0f;
//    if (U > 1.0f) U = 1.0f;
//
//    Intersection.x = LineStart.x + U * LineDir.x;
//    Intersection.y = LineStart.y + U * LineDir.y;
//    dist = magnitude(Point, Intersection);
//    return Intersection;
//}

// lineObstacle obstacles[MAX_OBS];
unsigned numObstacles = 0;

}
