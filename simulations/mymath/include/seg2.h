#ifndef SEG2_H
#define SEG2_H

#include <math.h>
#include <assert.h>
#include <iostream>
#include "vec2.h"

class Seg2
{
	public:
        Vec2 v1, v2;

        Seg2()
        {};

		inline Seg2(const Vec2 v1, const Vec2 v2) : v1 (v1), v2(v2)
		{

		};

		inline ~Seg2()
		{
        };

        bool doIntersect (Seg2& rSeg);

	protected:

	private:

};

#endif
