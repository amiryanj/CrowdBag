#ifndef VEC2F_H_INCLUDED
#define VEC2F_H_INCLUDED

#include <math.h>

namespace helbing3d {

class vec2f{
public:
    float x, y;

    vec2f(float x_, float y_) : x(x_), y(y_) {};
    vec2f() {x=0; y=0;};

    float distanceto(vec2f p) const{
		return sqrt((p.x-x)*(p.x-x) + (p.y-y)*(p.y-y));
	}

	float distanceto(float px, float py) const{
		return sqrt((px-x)*(px-x) + (py-y)*(py-y));
	}

	float distancetosqr(vec2f p) const{
		return (p.x-x)*(p.x-x) + (p.y-y)*(p.y-y);
	}

	float magnitude() const{
	    return sqrt(x*x + y*y);
	}

	float magnitudeSqr() const{
	    return x*x + y*y;
	}

	void negate(){
	    x *= -1;
	    y *= -1;
	}

	 vec2f operator-(const vec2f & a) const{
	    vec2f result = *this;
	    result.x -= a.x;
	    result.y -= a.y;
	    return result;
	}

	vec2f operator+(const vec2f & a) const{
	    vec2f result = *this;
	    result.x += a.x;
	    result.y += a.y;
	    return result;
	}

	vec2f operator*(float a) const{
	    vec2f result = *this;
	    result.x *= a;
	    result.y *= a;
	    return result;
	}

	vec2f operator/(float a) const{
	    vec2f result = *this;
	    result.x /= a;
	    result.y /= a;
	    return result;
	}
};

void normalize(vec2f& v);
float magnitude(vec2f Point1, vec2f Point2);
float magnitudeSqr(vec2f Point1, vec2f Point2);
float det(vec2f p, vec2f q);
float dot(vec2f p, vec2f q);
float absSq(vec2f q);

}

#endif // VEC2F_H_INCLUDED
