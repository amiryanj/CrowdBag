/*
 *  LineObstacle.h
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */

/*!
 *  @file       LineObstacle.h
 *  @brief      Contains the LineObstacle class.
 */

#pragma once
#include "util/Vector2D.h"

namespace TTC {

/*!
 *	@brief	A line segment obstacle class. 
 */
class LineObstacle 
  {
  public:
    /* Constructor. Constructs an obstacle.
      @param a The first endpoint of the obstacle. 
      @param b The second endpoint of the obstacle. */
	  LineObstacle(const Vector2D& a, const Vector2D& b);
		
	/*! Destructor. */
	~LineObstacle();
   
	/// @name Obstacle functionality
	//@{
	/// Returns the fist endpoint of the line segment.  
	Vector2D p1() const {return _p1;} 
	/// Returns the second endpoint of the line segment.  
	Vector2D p2() const {return _p2;}
	/// Returns the normal of the line obstacle.  
	Vector2D normal() const {return _normal;}
	//@}

  protected:
    /// The first endpoint of the obstacle. 
    Vector2D _p1;
    /// The second endpoint of the obstacle. 
    Vector2D _p2;
	/// The normal vector of the line obstacle. 
    Vector2D _normal;
  };

}