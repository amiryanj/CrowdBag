#include "../include/seg3.h"

bool Seg3::doIntersectXY (Seg3& rSeg)
{
    float precision = 10e-6f;
    float denominator_ua_ub = (rSeg.v2.y - rSeg.v1.y) * (v2.x - v1.x) - (rSeg.v2.x - rSeg.v1.x) * (v2.y - v1.y);

    if (denominator_ua_ub < precision && denominator_ua_ub > -precision)
    {
        // segment of lines are parallel
        return false;
    }

    float numerator_ua = (rSeg.v2.x - rSeg.v1.x) * (v1.y - rSeg.v1.y) - (rSeg.v2.y - rSeg.v1.y) * (v1.x - rSeg.v1.x);
    float numerator_ub = (v2.x - v1.x) * (v1.y - rSeg.v1.y) - (v2.y - v1.y) * (v1.x - rSeg.v1.x);

    float ua = numerator_ua / denominator_ua_ub;
    float ub = numerator_ub / denominator_ua_ub;

    if (ua < ub + precision && ua > ub - precision)
    {
        //line are coincident
        return true;
    }

    if (ua <= 1. && ua >= 0. && ub <= 1. && ua >= 0.) return true;

    return false;
}
