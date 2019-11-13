/*
 *  Vector2D.h
 *  
 *  
 *  All rights are retained by the authors and the University of Minnesota.
 *  Please contact sjguy@cs.umn.edu for licensing inquiries.
 *  
 *  Authors: Ioannis Karamouzas, Brian Skinner, and Stephen J. Guy
 *  Contact: ioannis@cs.umn.edu
 */

/*!
 *  @file       Vector2D.h
 *  @brief      Contains the Vector2D class; a two-dimensional vector class and related vector operations.
 */

#pragma once
#include <iostream>
#define _USE_MATH_DEFINES
#include <math.h>

namespace TTC {
/*! 
 *  \class Vector2D
 *  @brief A two-dimensional vector class. 
 *
 */
class Vector2D {

  public:
	  /*! The x-component of the vector. */
      float x;
      /*! The y-component of the vector. */
      float y;

	  /*! Constructs a null vector. */
	  inline Vector2D() { x = 0; y = 0; }
	
	  /*! 
		@brief Copy constructor. 
        @param v The vector to be copied. */
	  inline Vector2D(const Vector2D& v) { x = v.x; y = v.y; }

	  /*! 
		@brief Constructor. 
        @param _x The x-component of the new vector. 
        @param _y The y-component of the new vector. */
      inline Vector2D(float _x, float _y) { x = _x; y = _y; }

	/*! 
		@brief Vector addition. 
        @param v The vector to be added.
        @return The sum of the two vectors. */
    inline Vector2D operator+(const Vector2D& v) const { return Vector2D(x + v.x, y + v.y); }

	/*! 
		@brief Vector subtraction. 
        @param v The vector to be added.
        @return The vector difference. */
    inline Vector2D operator-(const Vector2D& v) const { return Vector2D(x - v.x, y - v.y); }
	
	/*! 
		@brief Dot product. 
		@param v The right hand side vector
		@return The dot product of the two vectors.  */
    inline float operator*(const Vector2D& v) const { return x * v.x + y * v.y; }
	
	 /*! 
	  @brief Scalar multiplication
	  @param a The scalar.  */
	  inline Vector2D operator*(const float a) const { return Vector2D(x * a, y * a); }

	  /*! 
	 @brief Scalar division
	 @param a The scalar.  */
    inline Vector2D operator/(const float a) const { return Vector2D(x / a, y / a); }

	/*! 
	@brief Cross product. 
    @param v The right hand side vector
    @return The cross product of the two vectors.  */
	inline float operator^(const Vector2D& v) const { return x*v.y - y*v.x;} 
	
	/*! 
		@brief Adds a 2d vector to the current vector.
        @param v The vector to be added. */
    inline void operator+=(const Vector2D& v) { x += v.x; y += v.y;}

	/*! 
		@brief Subtracts a 2d vector from the current vector.
        @param v The vector to be substracted. */
    inline void operator-=(const Vector2D& v) { x -= v.x; y -= v.y; }

	/*! 
		@brief Multiplies the vector by a scalar.
        @param a The scalar. */
    inline void operator*=(const float& a) { x *= a; y *= a;}

	/*! 
		@brief Divides the vector by a scalar.
        @param a The scalar.  */
    inline void operator/=(const float& a) { x /= a; y /= a;}

	/*! 
		@brief Vector equality. 
        @param v The right hand side vector
        @return True if the vectors are equal. False otherwise.  */
    inline bool operator==(const Vector2D& v) const { return (x == v.x && y == v.y); }

	/*! 
		@brief Vector inequality. 
        @param v The right hand side vector
        @return True if the two vector are not equal. False otherwise.  */
    inline bool operator!=(const Vector2D& v) const { return (x != v.x || y != v.y); }

	/*! @return This function normalizes the x and y coordinate. */
	inline void normalize() { float d=sqrtf(x*x+y*y); if(d>0) { x/=d; y/=d; }}

	/*! @return The magnitude of the vector.  */
	inline float length() const { return sqrtf(x*x+y*y);  }

	/*! @return The squared magnitude of the vector.  */
	inline float lengthSqr() const { return x*x+y*y; }

	/*! @return A vector perpendicular to the current vector.  */
	inline Vector2D perpendicular() const { return Vector2D(-y, x); }
	
  };


/*! 
	@brief Writes a vector to an output stream. 
    @param os The output stream
    @param v The vector
    @return The output stream.  */
	inline std::ostream& operator << (std::ostream& os, const Vector2D& v) {
		//return os << v.x << " " << v.y ;
		return os << "(" << v.x << "," << v.y << ")";
	}

/*! 
	@brief Normalization of a vector
	@param v A vector
    @return The normalized vector.  */
	inline Vector2D normalize(const Vector2D& v) { float d=sqrtf(v.x*v.x+v.y*v.y); if(d>0) return v/d; return v;}

/*! 
	@brief The dot product between two vectors.
	@param v1 A vector
    @param v2 A vector
    @return Returns the dot product between the two vectors */
	inline float dot(const Vector2D& v1, const Vector2D& v2) { return v1.x*v2.x + v1.y*v2.y ; }

/*! 
	@brief The cross product between two vectors
	@param v1 A vector
    @param v2 A vector
    @return Returns the cross product between the two vectors i.e determinant of the 2x2 matrix formed by using v1 as the first row and v2 as the second row. */
	inline float det(const Vector2D& v1, const Vector2D& v2) { return v1.x*v2.y - v1.y*v2.x; }

	 /*! 
	  @brief Scalar multiplication (left operator)
	  @param v A vector.
	  @param a The scalar.  
	  */
	inline Vector2D operator*(const float a, const Vector2D &v) {return Vector2D(v.x * a, v.y * a);}

	/*! 
		@brief Caps the magnitude of a vector to a maximum value. 
		@param force A force vector
		@param maxValue The maximum magnitude of the force. */
	inline void clamp (Vector2D &v, float maxValue) {
		float lengthV = v.length(); 
		if(lengthV > maxValue)
		{
			float mult = (maxValue / lengthV);
			v.x *=  mult;
			v.y *=  mult;
		}
	}
}