#ifndef CRAAL_RESOURCELISTER_H_
#define CRAAL_RESOURCELISTER_H_

#include "CraalIncludes.h"

#include "Calibration.h"
#include "Metric.h"
#include "Parser.h"
#include "Simulation.h"

namespace craal {

/**
 * The ResourceLister singleton class which stores all resources (@ref Calibration,
 * @ref Metric, @ref Parser or @ref Simulation) made available to the framework.
 * Resources are added using the
 * @ref ResourceLister::addCalibration(),
 * @ref ResourceLister::addMetric(),
 * @ref ResourceLister::addParser() and
 * @ref ResourceLister::addSimulation() methods.
 * A new instance of a resource is returned by the
 * @ref ResourceLister::calibration(),
 * @ref ResourceLister::metric(),
 * @ref ResourceLister::parser() and
 * @ref ResourceLister::simulation() methods.
 * 
 * Using any of this class' methods instantiates the singleton and
 * @ref ResourceLister::kill() deletes it.
 * 
 * The following example shows how to add Metric1 and Metric2 and then print the names
 * of all available metrics:
 * @code
 * 
 * ResourceLister::addMetric<Metric1>();
 * ResourceLister::addMetric<Metric2>();
 * 
 * Metric * metric;
 * for (int i = 0; i < ResourceLister::nbMetrics(); i++) {
 *     metric = ResourceLister::metric(i);
 *     std::cout<< metric->name<< std::endl;
 *     delete metric;
 * }
 * 
 * @endcode
 */
class ResourceLister
{
private:
	class CalibrationWrapperBase
	{public: virtual Calibration * create() = 0;};
	template <class T>
	class CalibrationWrapper : public CalibrationWrapperBase
	{public: typedef T t; virtual Calibration * create() {return new t();};};
	
	class MetricWrapperBase
	{public: virtual Metric * create() = 0;};
	template <class T>
	class MetricWrapper : public MetricWrapperBase
	{public: typedef T t; virtual Metric * create() {return new t();};};
	
	class ParserWrapperBase
	{public: virtual Parser * create() = 0;};
	template <class T>
	class ParserWrapper : public ParserWrapperBase
	{public: typedef T t; virtual Parser * create() {return new t();};};
	
	class SimWrapperBase
	{public: virtual Simulation * create() = 0;};
	template <class T>
	class SimWrapper : public SimWrapperBase
	{public: typedef T t; virtual Simulation * create() {return new t();};};
	
	std::vector<CalibrationWrapperBase*> g_calibrations;
	std::vector<MetricWrapperBase*> g_metrics;
	std::vector<ParserWrapperBase*> g_parsers;
	std::vector<SimWrapperBase*> g_simulations;
	
	std::vector<std::string> g_calibrationNames;
	std::vector<std::string> g_metricNames;
	std::vector<std::string> g_parserNames;
	std::vector<std::string> g_simulationNames;
	
	std::map<std::string, int> g_calibrationMap;
	std::map<std::string, int> g_metricMap;
	std::map<std::string, int> g_parserMap;
	std::map<std::string, int> g_simulationMap;
	
	static ResourceLister * _singleton;
	
	ResourceLister(){};
	virtual ~ResourceLister()
	{
		for (int i = 0; i < g_calibrations.size(); i++) {
			delete g_calibrations[i];
		}
		for (int i = 0; i < g_metrics.size(); i++) {
			delete g_metrics[i];
		}
		for (int i = 0; i < g_parsers.size(); i++) {
			delete g_parsers[i];
		}
		for (int i = 0; i < g_simulations.size(); i++) {
			delete g_simulations[i];
		}
	};
	
	static void p_create()
	{
		if (NULL == _singleton) _singleton = new ResourceLister;
	}

public:
	/** Delete the singleton instance. */
	static void kill()
	{
		if (NULL != _singleton) delete _singleton;
	}
	
	/** Get the number of available @ref Calibration objects. */
	static int nbCalibrations()
	{
		p_create();
		return _singleton->g_calibrations.size();
	};
	/** Get the number of available @ref Metric objects. */
	static int nbMetrics()
	{
		p_create();
		return _singleton->g_metrics.size();
	};
	/** Get the number of available @ref Parser objects. */
	static int nbParsers()
	{
		p_create();
		return _singleton->g_parsers.size();
	};
	/** Get the number of available @ref Simulation objects. */
	static int nbSimulations()
	{
		p_create();
		return _singleton->g_simulations.size();
	};
	
	/**
	 * Get the name of a @ref Calibration object.
	 * @param index of the @param Calibration object
	 */
	static std::string calibrationName(int s_i)
	{
		return _singleton->g_calibrationNames[s_i];
	};
	/**
	 * Get the name of a @ref Metric object.
	 * @param index of the @param Metric object
	 */
	static std::string metricName(int s_i)
	{
		return _singleton->g_metricNames[s_i];
	};
	/**
	 * Get the extension of a @ref Parser object.
	 * @param index of the @param Parser object
	 */
	static std::string parserExtension(int s_i)
	{
		return _singleton->g_parserNames[s_i];
	};
	/**
	 * Get the name of a @ref Simulation object.
	 * @param index of the @param Simulation object
	 */
	static std::string simulationName(int s_i)
	{
		return _singleton->g_simulationNames[s_i];
	};
	
	/**
	 * Get a new instance of a @ref Calibration object.
	 * @param index of the @param Calibration object
	 */
	static Calibration * calibration(int s_i)
	{
		p_create();
		return _singleton->g_calibrations[s_i]->create();
	};
	/**
	 * Get a new instance of a @ref Metric object.
	 * @param index of the @param Metric object
	 */
	static Metric * metric(int s_i)
	{
		p_create();
		return _singleton->g_metrics[s_i]->create();
	};
	/**
	 * Get a new instance of a @ref Parser object.
	 * @param index of the @param Parser object
	 */
	static Parser * parser(int s_i)
	{
		p_create();
		return _singleton->g_parsers[s_i]->create();
	};
	/**
	 * Get a new instance of a @ref Simulation object.
	 * @param index of the @param Simulation object
	 */
	static Simulation * simulation(int s_i)
	{
		p_create();
		return _singleton->g_simulations[s_i]->create();
	};
	
	/**
	 * Get a new instance of a @ref Calibration object.
	 * @param name of the @param Calibration object
	 */
	static Calibration * calibration(std::string s_name)
	{
		p_create();
		return _singleton->g_calibrations[_singleton->g_calibrationMap[s_name]]->create();
	};
	/**
	 * Get a new instance of a @ref Metric object.
	 * @param name of the @param Metric object
	 */
	static Metric * metric(std::string s_name)
	{
		p_create();
		return _singleton->g_metrics[_singleton->g_metricMap[s_name]]->create();
	};
	/**
	 * Get a new instance of a @ref Parser object.
	 * @param filename that one of the parsers should be able to parse
	 */
	static Parser * parser(std::string s_filename)
	{
		p_create();
		
		std::string filename = s_filename;
		size_t pos = filename.rfind(".");
		if (pos != std::string::npos) {
			filename = filename.substr(pos + 1);
		}
		
		return _singleton->g_parsers[_singleton->g_parserMap[filename]]->create();
	};
	/**
	 * Get a new instance of a @ref Simulation object.
	 * @param name of the @param Simulation object
	 */
	static Simulation * simulation(std::string s_name)
	{
		p_create();
		return _singleton->g_simulations[_singleton->g_simulationMap[s_name]]->create();
	};
	
	/** Add a new @ref Calibration class object. */
	template <class T>
	static void addCalibration()
	{
		p_create();
		T t;
		_singleton->g_calibrationNames.push_back(t.name);
		_singleton->g_calibrationMap[t.name] = _singleton->g_calibrations.size();
		_singleton->g_calibrations.push_back(new CalibrationWrapper<T>());
	};
	/** Add a new @ref Metric class object. */
	template <class T>
	static void addMetric()
	{
		p_create();
		T t;
		_singleton->g_metricNames.push_back(t.name);
		_singleton->g_metricMap[t.name] = _singleton->g_metrics.size();
		_singleton->g_metrics.push_back(new MetricWrapper<T>());
	};
	/** Add a new @ref Parser class object. */
	template <class T>
	static void addParser()
	{
		p_create();
		T t;
		_singleton->g_parserNames.push_back(t.getExension());
		_singleton->g_parserMap[t.getExension()] = _singleton->g_parsers.size();
		_singleton->g_parsers.push_back(new ParserWrapper<T>());
	};
	/** Add a new @ref Simulation class object. */
	template <class T>
	static void addSimulation()
	{
		p_create();
		T t;
		_singleton->g_simulationNames.push_back(t.name);
		_singleton->g_simulationMap[t.name] = _singleton->g_simulations.size();
		_singleton->g_simulations.push_back(new SimWrapper<T>());
	};
};

}

#endif /* CRAAL_RESOURCELISTER_H_ */
