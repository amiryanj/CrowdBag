#ifndef ANGLE_H
#define ANGLE_H

#include "mymath.h"
#include "vec2.h"

class Angle
{

public:
    Angle()
    {}
    ;

    Angle(Vec2 a, Vec2 b)
    {
        _aRad = b.direction() - a.direction();
        this->limitPi();
    }

    ~Angle()
    {}
    ;


    inline void limit2pi (void)
    {
        while (_aRad < 0.)
            _aRad += M_2_PI;
        while (_aRad > M_2_PI)
            _aRad -= M_2_PI;
    }

    inline void limitPi (void)
    {
        while (_aRad < -M_PI)
            _aRad += M_PI;
        while (_aRad > M_PI)
            _aRad -= M_PI;
    }

    inline Angle operator /= (const double rd)
    {
        _aRad /= rd;

        return *this;
    }

    inline double toDouble (void)
    {
        return _aRad;
    }

protected:

private:
    double _aRad;
};


#endif
