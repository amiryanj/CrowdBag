#include "../include/vec2f.h"

namespace helbing3d {

void normalize(vec2f& v){
    float len = sqrt(v.x*v.x+v.y*v.y);
    v.x /= len;
    v.y /= len;
}

float magnitude(vec2f Point1, vec2f Point2){
    return sqrtf( (Point2.x - Point1.x) * (Point2.x - Point1.x) + (Point2.y - Point1.y) * (Point2.y - Point1.y));
}

float magnitudeSqr(vec2f Point1, vec2f Point2){
    return (Point2.x - Point1.x) * (Point2.x - Point1.x) + (Point2.y - Point1.y) * (Point2.y - Point1.y);
}

float det(vec2f p, vec2f q) { return p.x*q.y - p.y*q.x; }
float dot(vec2f p, vec2f q) { return p.x*q.x + p.y*q.y; }
float absSq(vec2f q) {return dot(q,q);}

}
