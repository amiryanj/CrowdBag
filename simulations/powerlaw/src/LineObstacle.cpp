/*
 *  LineObstacle.cpp
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */

#include "../include/LineObstacle.h"

namespace TTC {

	LineObstacle::LineObstacle(const Vector2D& a, const Vector2D& b) :
		_p1(a), _p2(b)
	{
		_normal = normalize((_p2-_p1).perpendicular());
	}

	LineObstacle::~LineObstacle()
	{
			
	}
}
