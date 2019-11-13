#ifndef CRAAL_PDF_H_
#define CRAAL_PDF_H_

#include "CraalIncludes.h"

namespace craal {

/**
 * The PDF (probability density function) class.
 * This class defines bounded distributions from which to randomly draw values.
 * Currently it implements uniform and normal distributions.
 */
class PDF
{
public:
	/** The distribution type, currently either uniform or normal. */
	enum DISTRIBUTION
	{
		UNIFORM,
		NORMAL
	};

public:
	/** Lower bound. */
	float g_lowerBound;
	/** Higher bound. */
	float g_higherBound;
	/** First additional parameter, required for normal distributions (mean). */
	float g_param1;
	/** Second additional parameter, required for normal distributions (standard deviation). */
	float g_param2;
	/** Precision of the drawing (sampling of the interval defined by the lower and higher bounds. */
	int g_precision;
	
	/** Distribution type. */
	DISTRIBUTION g_distrib;
	
	/** Constructor. */
	PDF();
	/**
	 *Constructor.
	 * @param s_lowerBound sets @ref PDF::g_lowerBound
	 * @param s_higherBound sets @ref PDF::g_higherBound
	 * @param s_distrib sets @ref PDF::g_distrib
	 * @param s_param1 sets @ref PDF::g_param1
	 * @param s_param2 sets @ref PDF::g_param2
	 */
	PDF(float s_lowerBound, float s_higherBound, DISTRIBUTION s_distrib = UNIFORM, float s_param1 = 0, float s_param2 = 0);
	/** Destructor. */
	virtual ~PDF();
	
	/** Draw a new random value. */
	float get();
	/**
	 *Draw a new random value (with normal distribution) with temporary additional parameters.
	 * @param s_param1 temporarily replaces @ref PDF::g_param1
	 * @param s_param2 temporarily replaces @ref PDF::g_param2
	 */
	float getCustom(float s_param1, float s_param2);
	/**
	 * Random (with uniform distribution) yes or no.
	 * @param s_p test value, between 0 and 1
	 * @returns @code r <= s_p*(float)RAND_MAX; @endcode
	 */
	static bool yesNo(float s_p);
	/**
	 * Get a random (with uniform distribution) integer between 0 and s_i.
	 * @param s_i maximum integer + 1
	 * @return @code rand() % s_i; @endcode
	 */
	static int getInt(int s_i);
	
	/**
	 *Initializes the random seed.
	 * @code srand ( time(NULL) ); @endcode
	 */
	static void init()
		{srand ( time(NULL) );};
};

}

#endif /* CRAAL_PDF_H_ */
