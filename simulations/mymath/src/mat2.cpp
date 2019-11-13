#include <string.h>
#include <math.h>

#include "../include/mat2.h"

void Mat2::setIdentity(void)
{
    memset(m, 0, 4*sizeof(float));
    m[0][0] = 1;
    m[1][1] = 1;
}

void Mat2::setRot(float angle)
{
    memset(m, 0, 4*sizeof(float));
    m[0][0] = cos(angle);
    m[0][1] = -sin(angle);
    m[1][0] = -m[0][1];
    m[1][1] = m[0][0];
}

