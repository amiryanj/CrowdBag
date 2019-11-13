#ifndef SEG3_H
#define SEG3_H

#include <math.h>
#include <assert.h>
#include <iostream>
#include "vec3.h"

class Seg3
{
	public:
        Vec3 v1, v2;

        Seg3()
        {};

		inline Seg3(const Vec3 v1, const Vec3 v2) : v1 (v1), v2(v2)
		{

		};

		inline ~Seg3()
		{
        };

        bool doIntersectXY (Seg3& rSeg);

	protected:

	private:

};

#endif
