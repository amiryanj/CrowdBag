#include "../include/mymath.h"

float my_atan2(float y, float x)
{
    float coeff_1 = PI / 4.f;
    float coeff_2 = 3.f * coeff_1;
    float abs_y = std::abs(y);
    float angle;
    if (x >= 0.f)
    {
        float r = (x - abs_y) / (x + abs_y);
        angle = coeff_1 - coeff_1 * r;
    }
    else
    {
        float r = (x + abs_y) / (abs_y - x);
        angle = coeff_2 - coeff_1 * r;
    }
    return y < 0.f ? -angle : angle;
}

