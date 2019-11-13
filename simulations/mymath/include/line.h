#ifndef LINE_H
#define LINE_H

#include <math.h>
#include "vec2.h"

#define LINE_EPS3 0.001

class Line
{
public: //
    float a, b, c;

    //constructors
    Line ()
    {}

    inline Line (const Vec2 direction, const Vec2 point)
            : a (direction.y),
            b (- direction.x),
            c(-direction.y * point.x + direction.x * point.y)
    {}

    inline float distanceToPoint(Vec2 point) const
    {
        return fabs (a*point.x + b*point.y + c) / sqrt(a*a+b*b);
    }

    inline void defineBy2Points (const Vec2& A, const Vec2& B)
    {
        a = (B.y - A.y);
        b = (A.x - B.x);
        c = - a * A.x - b * A.y;
    }

    inline Vec2 intersection (const Line& l)
    {
        Vec2 sol;
        if (fabs(a) > fabs(b))
        {
            sol.y = (l.a*c - a*l.c) / (a*l.b - l.a*b);
            sol.x = -( b*sol.y + c) / a;
        }
        else
        {
            sol.x = (l.b*c - b*l.c) / (b*l.a - l.b*a);
            sol.y = -(a*sol.x + c) / b;
        }
        return sol;
    }

    inline void defineByPerpLinePoint (const Line&l, const Vec2& p)
    {
        a = l.b;
        b = - l.a;
        c = (-a * p.x + -b * p.y);
    }

    inline Vec2 xAxisIntersection ()
    {
        Vec2 sol(0.,0.);

        sol.x = -c/a;

        return sol;
    }

    inline Vec2 yAxisIntersection ()
    {
        Vec2 sol(0.,0.);
        sol.y = -c/b;

        return sol;
    }

    //destructors
    inline ~Line()
    {}

protected:

private:

};

#endif

