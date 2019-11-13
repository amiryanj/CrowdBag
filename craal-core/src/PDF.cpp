#include "../include/PDF.h"

namespace craal {

PDF::PDF() :
	g_precision(-1)
{}
PDF::PDF(float s_lowerBound, float s_higherBound, DISTRIBUTION s_distrib, float s_param1, float s_param2) :
	g_lowerBound(s_lowerBound), g_higherBound(s_higherBound), g_distrib(s_distrib), g_param1(s_param1), g_param2(s_param2), g_precision(1000000)
{
	g_precision = CRAAL_MIN(g_precision, RAND_MAX);
}

PDF::~PDF()
{}

float PDF::get()
{
	if (g_precision == -1) return 0;
	float r = 0;
	
	switch (g_distrib)
	{
		case UNIFORM:
			r = rand() % (g_precision)/((float)g_precision-1);
			r = r*(g_higherBound-g_lowerBound)+g_lowerBound;
			break;

		case NORMAL: // Box-Muller method
			float r1 = rand() % (g_precision)/((float)g_precision-1);
			float r2 = rand() % (g_precision)/((float)g_precision-1);
			
			r = sqrt(-2*log(r1))*cos((double)(2.0*CRAAL_PI*r2));
			r = r*g_param2+g_param1;
			r = CRAAL_MIN(CRAAL_MAX(r, g_lowerBound), g_higherBound);
			break;
	}

	return r;
}

float PDF::getCustom(float s_param1, float s_param2)
{
	float r = 0;
	
	float r1 = rand() % (g_precision)/((float)g_precision-1);
	float r2 = rand() % (g_precision)/((float)g_precision-1);
	
	r = sqrt(-2*log(r1))*cos((double)(2.0*CRAAL_PI*r2));
	r = r*s_param2+s_param1;
	r = CRAAL_MIN(CRAAL_MAX(r, g_lowerBound), g_higherBound);

	return r;
}

bool PDF::yesNo(float s_p)
{
	int r = rand();
	
	return r <= s_p*(float)RAND_MAX;
}
int PDF::getInt(int s_i)
{
	return rand() % s_i;
}

}
