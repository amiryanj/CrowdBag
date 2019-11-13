#ifndef VEC2_H
#define VEC2_H

#include <cmath>
#include <assert.h>
#include <iostream>

class Vec2
{
public: //
    float x, y;

    //constructors
    Vec2 () : x (0.0f), y (0.0f)
    {}

    inline Vec2 (const float coordX, const float coordY)
            : x (coordX), y (coordY)
    {}

    inline Vec2 (const double coordX, const double coordY)
            : x ((float)coordX), y ((float) coordY)
    {}

    inline Vec2 (const double * pCoord)
            : x ((float) pCoord[0]), y ((float) pCoord [1])
    {}

    inline Vec2 (const float * pCoord)
            : x (pCoord[0]), y (pCoord [1])
    {}

    //destructors
    inline ~Vec2()
    {}


    inline Vec2 operator - ( const Vec2& rkVec ) const
    {
        return Vec2 (
                   x - rkVec.x,
                   y - rkVec.y
               );
    }

    inline Vec2 operator -= (const Vec2& rkVec)
    {
        x-= rkVec.x;
        y-= rkVec.y;

        return *this;
    }

    inline Vec2 operator += (const Vec2& rkVec)
    {
        x+= rkVec.x;
        y+= rkVec.y;

        return *this;
    }

    inline Vec2 operator *= (const float fScalar)
    {
        x*= fScalar;
        y*= fScalar;

        return *this;
    }


    inline Vec2 operator * ( const float fScalar ) const
    {
        return Vec2(
                   x * fScalar,
                   y * fScalar);
    }
//
//    inline Vec2 operator * ( const Vec2 rkVec ) const
//    {
//        return Vec2(
//                   x * rkVec.x,
//                   y * rkVec.y);
//    }

    inline float operator * ( const Vec2 rkVec ) const
    {
        return (x * rkVec.x + y * rkVec.y);
    }

    inline Vec2 operator / ( const float fScalar ) const
    {
        const float invS = 1.0f / fScalar;
        return Vec2(x * invS, y * invS);
    }

    inline Vec2 operator + ( const Vec2& rkVector ) const
    {
        return Vec2(
                   x + rkVector.x,
                   y + rkVector.y);
    }

    inline float normalize ()
    {
        float length = sqrt (x*x + y*y);
        if (length > 1e-8)
        {
            float inv_length = 1.f/length;
            x *= inv_length;
            y *= inv_length;
        }
        return length;
    }

    inline void maximize (const Vec2& rkVec)
    {
        if (x < rkVec.x)
            x = rkVec.x;
        if (y < rkVec.y)
            y = rkVec.y;
    }

    inline void minimize (const Vec2& rkVec)
    {
        if (x > rkVec.x)
            x = rkVec.x;
        if (y > rkVec.y)
            y = rkVec.y;
    }

    inline float operator [] ( const size_t i ) const
    {
        assert( i < 2 );

        return *(&x+i);
    }

    inline friend std::ostream& operator<<(std::ostream& s, Vec2& rkVec)
    {
        s << rkVec.x << " " << rkVec.y;
        return s;
    }

    inline float distance2 (const Vec2& rkVec) const
    {
        return ( (rkVec.x - x)*(rkVec.x - x) + (rkVec.y - y)*(rkVec.y - y) );
    }

    inline float distance (const Vec2& rkVec)
    {

        return (sqrt(distance2(rkVec)));
    }

    inline float longueur (void) const
    {
        return sqrt( x*x + y*y );
    }

    inline float direction (void) const
    {
        return atan2(y,x);
    }

protected:

private:

};

#endif

