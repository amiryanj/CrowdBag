#include "../include/Calibration.h"

namespace craal {

Calibration::Calibration() :
	g_nbParams(0), g_score(-1), g_verbose(false), g_metric(0)
{}

Calibration::~Calibration()
{
// 	if (g_metric) delete g_metric;
}

void Calibration::setRecord(Record * s_record)
{
	g_record = s_record;
	
	std::vector<PDF> pdf = g_record->getSimulation()->getPdfs();
	
	g_nbParams = 0;
	g_params.clear();
	
	for (int i = 0; i < pdf.size(); i++) {
		addParameter(pdf[i]);
	}
}

void Calibration::setMetric(Metric * s_metric)
{
	g_metric = s_metric;
}

void Calibration::addParameter(PDF s_pdf)
{
	Parameter p;
	
	for (int i = 0; i < g_record->getNPedestrianSimulated(); i++) {
		p.value = g_record->getParam(i, g_nbParams);
		p.tries = 20;
		p.pdf = s_pdf;
		p.best = *(p.value);

		g_params.push_back(p);
	}
	g_nbParams++;
}

void Calibration::printParams(std::ostream & s_stream)
{
	int nped = g_record->getNPedestrianSimulated();
	
	float * paramsavg = new float[g_nbParams];
	float * paramssd = new float[g_nbParams];
	
	for (int i = 0; i < g_nbParams; i++) {
		paramsavg[i] = 0;
		
		for (int j = 0; j < nped; j++) {
			paramsavg[i] += (g_params[i*nped+j].best);
		}
		
		paramsavg[i] /= ((float)nped);
		paramssd[i] = 0;
		
		for (int j = 0; j < nped; j++) {
			paramssd[i] += (paramsavg[i]-(g_params[i*nped+j].best))*(paramsavg[i]-(g_params[i*nped+j].best));
		}
		
		paramssd[i] /= ((float)nped);
		paramssd[i] = sqrt(paramssd[i]);
	}

	s_stream<< "\n#--BEST PARAMETERS--\n\n";
	s_stream<< "%%cost\n    "<< g_score<< std::endl;
	for (int i = 0; i < g_nbParams; i++) {
// 		s_stream<< "%%parameter_"<< i<< "\n    "<< paramsavg[i]<< " "<< paramssd[i]<< std::endl;
		s_stream<< std::endl<< "# "<< g_record->getSimulation()->getConfigurables()[i]<< std::endl;
		s_stream<< "\%parameter_"<< i<< std::endl;
		for (int j = 0; j < nped; j++) {
			s_stream<< "    "<< g_params[i*nped+j].best<< std::endl;
		}
	}
	
	delete [] paramsavg;
	delete [] paramssd;
}

}
