#ifndef CRAAL_CALIBRATION_H_
#define CRAAL_CALIBRATION_H_

#include "CraalIncludes.h"

#include "Configurable.h"
#include "Metric.h"
#include "PDF.h"
#include "Record.h"

namespace craal {

/**
 * The base Calibration class.
 * An optimization algorithm must extend this class in order to be usable
 * by the framework.
 * 
 * It is used in the following way:
 *  - @ref Calibration::setRecord() to set the @ref Record object
 *  - @ref Calibration::setMetric() to set the @ref Metric object
 *  - @ref Calibration::calibrate() to calibrate the @ref Calibration
 *    object contained in the @ref Record object
 * 
 * The only thing to implement here is the @ref Calibration::calibrate() method.
 * 
 * See @ref IMPLEMENT_CALIB for examples on how to implement and combine calibration algorithms.
 * 
 * @todo Update the API (remove or at least improve over the existing @ref Calibration::Parameter
 * struct). Documented entries will probably stay.
 */
class Calibration : public Configurable
{
public:
	/** Temporary state of the internal representation of a parameter. */
	struct Parameter
	{
		float * value;
		int tries;
		PDF pdf;
		float best;
	};
	
	std::vector<Parameter> g_params;
	/** The @ref Record object. */
	Record * g_record;
	/** The number of parameters. */
	int g_nbParams;
	/** Used to store the @ref Metric object's score after calibration. */
	float g_score;
	/** The verbose flag. */
	bool g_verbose;
	/** The @ref Metric object. */
	Metric * g_metric;

public:
	/** Constructor. */
	Calibration();
	/** Destructor. */
	virtual ~Calibration();
	
	/**
	 * Set the @ref Record object.
	 * @param s_record the @ref Record object.
	 */
	void setRecord(Record * s_record);
	
	/**
	 * Set the @ref Metric object.
	 * @param s_metric the @ref Metric object.
	 */
	void setMetric(Metric * s_metric);
	
	void addParameter(PDF s_pdf);
	/**
	 * Prints the computed parameter values with the associated @ref Metric cost.
	 * @param s_stream std::ostream indicating where to print, could be a file (default = std::cout)
	 */
	void printParams(std::ostream & s_stream = std::cout);
	
	/**
	 * The method which must be implemented by any metric algorithm.
	 * Should set the values of @ref Calibration::Parameter::best in the
	 * @ref Calibration::g_params list.
	 */
	virtual void calibrate() = 0;
	
	/** Resets the @ref Calibration::g_params list. */
	inline void reset() {g_params.clear(); g_nbParams = 0;};
	/** 
	 * Toggles verbose (turned off by default).
	 * @param s_verbose verbose boolean
	 */
	inline void verbose(bool s_verbose) {g_verbose = s_verbose;};
	/** Returns the @ref Metric score obtained after calibration. */
	inline float getScore() {return g_score;};
};

}

#endif /* CRAAL_CALIBRATION_H_ */
