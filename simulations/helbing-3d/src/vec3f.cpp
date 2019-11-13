#include "../include/vec3f.h"

namespace helbing3d {

void normalize(vec3f& v){
    float len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    v.x /= len;
    v.y /= len;
	v.z /= len;
}
vec3f normalize(vec3f v){
    float len = sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
	
	vec3f ret = v;
	
    ret.x /= len;
    ret.y /= len;
	ret.z /= len;
	
	return ret;
}

float magnitude(vec3f Point1, vec3f Point2){
    return sqrtf( (Point2.x - Point1.x) * (Point2.x - Point1.x) + (Point2.y - Point1.y) * (Point2.y - Point1.y) + (Point2.z - Point1.z) * (Point2.z - Point1.z));
}

float magnitudeSqr(vec3f Point1, vec3f Point2){
    return (Point2.x - Point1.x) * (Point2.x - Point1.x) + (Point2.y - Point1.y) * (Point2.y - Point1.y) + (Point2.z - Point1.z) * (Point2.z - Point1.z);
}

float det(vec3f p, vec3f q) { 
	return sqrtf ((p.y*q.z - p.z*q.y) * (p.y*q.z - p.z*q.y) + (q.x*p.z - p.x*q.z) * (q.x*p.z - p.x*q.z) + (p.x*q.y - p.y*q.x) * (p.x*q.y - p.y*q.x)); }
float dot(vec3f p, vec3f q) { return p.x*q.x + p.y*q.y + p.z*q.z; }

vec3f cross(vec3f p, vec3f q) { return vec3f(p.y*q.z - p.z*q.y,  p.z*q.x - p.x*q.z, p.x*q.y - p.y*q.x); }

float absSq(vec3f q) {return dot(q,q);}

}
