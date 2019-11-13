#ifndef CRAAL_METRIC_H_
#define CRAAL_METRIC_H_

#include "CraalIncludes.h"

#include "Configurable.h"
#include "Record.h"

namespace craal {

/**
 * The base Metric class.
 * A metric algorithm must extend (virtual) this class in order to be usable
 * by the framework.
 * 
 * The only thing to implement here is the @ref Metric::evaluate() method.
 * It is, however, important to note that it is the metric that decides how
 * and when to compute the paths in the simulation (in @ref Metric::evaluate()):
 *  - For example, a metric could focus on comparing the complete trajectories
 *    of a simulator with the trajectories from the example data. In this case,
 *    at the beginning of the @ref Metric::evaluate() method, the
 *    @ref Record::computePath() method should be called.
 *  - In another case, the metric could compare reactions of the real and simulated
 *    pedestrians by repositionning the simulation on the real data and compute
 *    only one frame at each timestep. This would be done by calling
 *    @ref Record::computeNext() when necessary inside @ref Metric::evaluate().
 * 
 * See @ref IMPLEMENT_METRIC for examples on how to implement and combine metrics.
 */
class Metric : public Configurable
{
public:
	/** Constructor. */
	Metric();
	/** Destructor. */
	virtual ~Metric();
	
	/**
	 * This function computes a score based on data and the simulation.
	 * @param s_record the @ref Record object which contains the experimental
	 * (@ref Experiment) and simulation (@ref Simulation) data.
	 * @return a float value representing the score, a lower score should mean
	 * the simulation more closely matches reference data (ideally 0)
	 */
	virtual float evaluate(Record * s_record) = 0;

    /**
     * This function computes a score based on data and the simulation.
     * @param s_record the @ref Record object which contains the experimental
     * @param s_scores the computed scores for each of the agents in an array
     * (@ref Experiment) and simulation (@ref Simulation) data.
     * lower scores should mean the simulation more closely matches reference data (ideally 0)
     */
    virtual void evaluate(Record * s_record, std::vector<float> & s_scores) { };
};

}

#endif /* CRAAL_METRIC_H_ */
