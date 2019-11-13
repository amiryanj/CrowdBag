#ifndef VEC3_H
#define VEC3_H

#include <math.h>
#include <assert.h>
#include <iostream>

class Vec3
{
public: //
    float x, y, z;

    //constructors
    Vec3 () : x (0.0), y (0.0), z (0.0)
    {}

    inline Vec3 (const float coordX, const float coordY, const float coordZ)
            : x (coordX), y (coordY), z (coordZ)
    {}

    inline Vec3 (const double coordX, const double coordY, const double coordZ)
            : x ((float)coordX), y ((float) coordY), z ((float) coordZ)
    {}

    inline Vec3 (const double * pCoord)
            : x ((float) pCoord[0]), y ((float) pCoord [1]), z ((float) pCoord[2])
    {}

    inline Vec3 (const float * pCoord)
            : x (pCoord[0]), y (pCoord [1]), z (pCoord[2])
    {}

    //destructors
    inline ~Vec3()
    {}


    inline Vec3 operator - ( const Vec3& rkVec ) const
    {
        return Vec3 (
                   x - rkVec.x,
                   y - rkVec.y,
                   z - rkVec.z
               );
    }

    inline Vec3 operator -= (const Vec3& rkVec)
    {
        x-= rkVec.x;
        y-= rkVec.y;
        z-= rkVec.z;

        return *this;
    }

    inline Vec3 operator += (const Vec3& rkVec)
    {
        x+= rkVec.x;
        y+= rkVec.y;
        z+= rkVec.z;

        return *this;
    }

    inline Vec3 operator * ( const float fScalar ) const
    {
        return Vec3(
                   x * fScalar,
                   y * fScalar,
                   z * fScalar);
    }

    inline Vec3 operator / ( const float fScalar ) const
    {
        return Vec3(
                   x / fScalar,
                   y / fScalar,
                   z / fScalar);
    }

    inline Vec3 operator + ( const Vec3& rkVector ) const
    {
        return Vec3(
                   x + rkVector.x,
                   y + rkVector.y,
                   z + rkVector.z);
    }

    inline float operator * ( const Vec3 rkVec ) const
    {
        return (x * rkVec.x + y * rkVec.y + z * rkVec.z);
    }

    inline float normalize ()
    {
        float length = sqrt (x*x + y*y + z*z);
        if (length > 1e-8)
        {
            float inv_length = 1.f/length;
            x *= inv_length;
            y *= inv_length;
            z *= inv_length;
        }
        return length;
    }

    inline Vec3 crossProduct (const Vec3& rkVec) const
    {
        return Vec3 (
                   y * rkVec.z - z * rkVec.y,
                   z * rkVec.x - x * rkVec.z,
                   x * rkVec.y - y * rkVec.x
               );
    }

    inline void maximize (const Vec3& rkVec)
    {
        if (x < rkVec.x)
            x = rkVec.x;
        if (y < rkVec.y)
            y = rkVec.y;
        if (z < rkVec.z)
            z = rkVec.z;
    }

    inline void minimize (const Vec3& rkVec)
    {
        if (x > rkVec.x)
            x = rkVec.x;
        if (y > rkVec.y)
            y = rkVec.y;
        if (z > rkVec.z)
            z = rkVec.z;
    }

    inline void rotateX270 (void)
    {
        float buffer = y;
        y = z;
        z = -buffer;
    }

    inline float operator [] ( const size_t i ) const
    {
        assert( i < 3 );

        return *(&x+i);
    }

    inline friend std::ostream& operator<<(std::ostream& s, Vec3& rkVec)
    {
        s << rkVec.x << " " << rkVec.y << " " << rkVec.z ;
        return s;
    }

    inline float distance2 (const Vec3& rkVec) const
    {
        return ( (rkVec.x - x)*(rkVec.x - x) + (rkVec.y - y)*(rkVec.y - y) + (rkVec.z - z)*(rkVec.z - z) );
    }

    inline float longueurXY (void) const
    {
        return sqrt( x*x + y*y );
    }


    inline float distance (const Vec3& rkVec)
    {

        return (sqrt(distance2(rkVec)));
    }

    inline float distanceXY (const Vec3& rkVec)
    {

        return (sqrt((rkVec.x - x)*(rkVec.x - x) + (rkVec.y - y)*(rkVec.y - y)));
    }

protected:

private:

};

#endif

