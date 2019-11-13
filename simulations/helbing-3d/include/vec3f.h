#ifndef VEC3F_H_INCLUDED
#define VEC3F_H_INCLUDED

#include <math.h>

namespace helbing3d {

class vec3f{
public:
    float x, y, z;

    vec3f(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {};
    vec3f() {x=0; y=0; z=0;};

    float distanceto(vec3f p) const{
		return sqrt((p.x-x)*(p.x-x) + (p.y-y)*(p.y-y) + (p.z-z)*(p.z-z));
	}

	float distanceto(float px, float py, float pz) const{
		return sqrt((px-x)*(px-x) + (py-y)*(py-y) + (pz-z)*(pz-z));
	}

	float distancetosqr(vec3f p) const{
		return (p.x-x)*(p.x-x) + (p.y-y)*(p.y-y) + (p.z-z)*(p.z-z);
	}

	float magnitude() const{
	    return sqrt(x*x + y*y + z*z);
	}

	float magnitudeSqr() const{
	    return x*x + y*y + z*z;
	}

	void negate(){
	    x *= -1;
	    y *= -1;
		z *= -1;
	}

	 vec3f operator-(const vec3f & a) const{
	    vec3f result = *this;
	    result.x -= a.x;
	    result.y -= a.y;
		result.z -= a.z;
	    return result;
	}

	vec3f operator+(const vec3f & a) const{
	    vec3f result = *this;
	    result.x += a.x;
	    result.y += a.y;
		result.z += a.z;
	    return result;
	}

	vec3f operator*(float a) const{
	    vec3f result = *this;
	    result.x *= a;
	    result.y *= a;
		result.z *= a;
	    return result;
	}

	vec3f operator/(float a) const{
	    vec3f result = *this;
	    result.x /= a;
	    result.y /= a;
		result.z /= a;
	    return result;
	}
};

void normalize(vec3f& v);
vec3f normalize(vec3f v);
float magnitude(vec3f Point1, vec3f Point2);
float magnitudeSqr(vec3f Point1, vec3f Point2);
float det(vec3f p, vec3f q);
float dot(vec3f p, vec3f q);
vec3f cross(vec3f p, vec3f q);

float absSq(vec3f q);

}

#endif // VEC3F_H_INCLUDED
