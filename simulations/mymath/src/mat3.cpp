#include "../include/mat3.h"
#include <string.h>
#include <math.h>

void Mat3::setIdentity(void)
{
    memset(m, 0, 9*sizeof(float));
    m[0][0] = 1;
    m[1][1] = 1;
    m[2][2] = 1;
}

void Mat3::setXRot(float angle)
{
    memset(m, 0, 9*sizeof(float));

    m[0][0] = 1.0;
    m[1][1] = cos(angle);
    m[1][2] = -sin(angle);
    m[2][1] = -m[1][2];
    m[2][2] = m[1][1];
}

void Mat3::setYRot(float angle)
{
    memset(m, 0, 9*sizeof(float));
    m[0][0] = cos(angle);
    m[0][2] = sin(angle);
    m[1][1] = 1.0;
    m[2][0] = -m[0][2];
    m[2][2] = m[0][0];
}

void Mat3::setZRot(float angle)
{
    memset(m, 0, 9*sizeof(float));
    m[0][0] = cos(angle);
    m[0][1] = -sin(angle);
    m[1][0] = -m[0][1];
    m[1][1] = m[0][0];
    m[2][2] = 1.;
}
