/*
 *  Definitions.h
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */

/*!
 *  @file       Definitions.h
 *  @brief      Defines global functions used throughout the solution.
 */
#pragma once
#include "Vector2D.h"
#include <vector>
#include <algorithm>

namespace TTC {


	#define _INFTY 9e9f

	#define _EPSILON 0.00001f

	#define _M_PI 3.14159265358979323846f


	/*! 
	@brief The square  of a value.
     @param a A scalar value
     @return Returns the square of the scalar.
    */
	inline float Sqr(float a) {return a*a;}
	
	
 /*! 
	    @brief Determine the closest point on a line segment given a test point.
		@param line_start The start point of the line segment.
		@param line_end The end point of the line segment.
		@param p The test point.
		@return The closest point on the line segment. */
	inline Vector2D	closestPointLineSegment (const Vector2D &line_start,const Vector2D &line_end, const Vector2D &p)
	{
		float dota =(p-line_start) * (line_end - line_start);
		if (dota <= 0) // point line_start is closest to p
			return line_start;

		float dotb = (p-line_end)*(line_start - line_end);
		if (dotb <= 0) // point line_end is closest to p
			return line_end;
	
		// find closest point
		float slope = dota/(dota+dotb);
		return line_start + (line_end - line_start)*slope;
	}
	
}