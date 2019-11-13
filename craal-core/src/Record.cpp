#include "../include/Record.h"

namespace craal {

Record::Record() :
	g_experiment(0), g_simulation(0),
	g_mask(0), g_map(0), g_invmap(0),
	g_nPedestrian(0), g_nPedestrianSimulated(0),
	g_nMeasure(0), g_nParams(0)
{
	addConfigurable("Save trajectories", new Type_string, &g_saveTrajectories);
	addConfigurable("Save parameters", new Type_string, &g_saveParameters);
	addConfigurable("Load parameters", new Type_string, &g_loadParameters);
	
	config("Save trajectories", "");
	config("Save parameters", "");
	config("Load parameters", "");
	
	name = "Record";
}

Record::~Record()
{
	if (g_mask) delete [] g_mask;
	if (g_map) delete [] g_map;
	if (g_invmap) delete [] g_invmap;
}

void Record::p_initSim()
{	
	if (g_experiment->isRealData()) {
		for (int i = 0; i < g_nPedestrian; i++) {
			setSimPosition(i,
				getExpPositionx(i, 0),
				getExpPositiony(i, 0));
			setSimVelocity(i,
				getExpVelocityx(i, 0),
				getExpVelocityy(i, 0));
			setSimGoal(i,
				getExpPositionx(i, g_nMeasure-1),
				getExpPositiony(i, g_nMeasure-1));
		}
		sync(0);
	}
	else {
		for (int i = 0; i < g_nPedestrian; i++) {
			setSimPosition(i,
				g_experiment->getWaypointx(i, 0),
				g_experiment->getWaypointy(i, 0));
			setSimVelocity(i, 0, 0);
			setSimGoal(i,
				g_experiment->getWaypointx(i, 1),
				g_experiment->getWaypointy(i, 1));
		}
	}
}

void Record::p_computePathRealData()
{
	g_simulation->clear();
	
	for (int i = 0; i < g_nPedestrian; i++) {
		g_simulation->addState(g_map[i],
			getExpPositionx(i, 0),
			getExpPositiony(i, 0),
			getExpVelocityx(i, 0),
			getExpVelocityy(i, 0));
		setSimPosition(i,
			getExpPositionx(i, 0),
			getExpPositiony(i, 0));
		setSimVelocity(i,
			getExpVelocityx(i, 0),
			getExpVelocityy(i, 0));
	}
	
	for (int i = 0; i < g_nMeasure-1; i++) {
		g_simulation->doStep(g_experiment->getTime(i+1)-g_experiment->getTime(i));
		
		for (int j = 0; j < g_nPedestrian; j++) {
			if (g_mask[j]) {
				g_simulation->addState(g_map[j],
					getSimPositionxNext(j),
					getSimPositionyNext(j),
					getSimVelocityxNext(j),
					getSimVelocityyNext(j));
			
			}
			else {
				g_simulation->addState(g_map[j],
					getExpPositionx(j, i+1),
					getExpPositiony(j, i+1),
					getExpVelocityx(j, i+1),
					getExpVelocityy(j, i+1));
				setSimPosition(j,
					getExpPositionx(j, i+1),
					getExpPositiony(j, i+1));
				setSimVelocity(j,
					getExpVelocityx(j, i+1),
					getExpVelocityy(j, i+1));
			}
		}
	}
	
	g_simulation->resetNMeasure();
}

void Record::p_computePathFakeData(bool s_save)
{
	g_simulation->clear();
	int * index = new int[g_nPedestrian];
// 	float * oldVelx = new float[g_nPedestrian];
// 	float * oldVely = new float[g_nPedestrian];
	bool loop = true;
	float tmp1, tmp2, tmp;
	int mes = 0;
	int counter = 0;
	float x, y, vx, vy;
	
// 	if (s_save) {
// 		std::ofstream filee;
// 		filee.open("params.debug");
// 		
// 		for (int i = 0; i < g_nPedestrian; i++) {
// 			for (int j = 0; j < getSimulation()->getNParam(); j++) {
// 				filee<< (*getSimulation()->getParam(i, j))<< std::endl;
// 			}
// 		}
// 		
// 		filee.close();
// 		
// 		for (int i = 0; i < g_nPedestrian; i++) {
// 			std::cout<< g_experiment->getNWaypoint(i)<< std::endl;
// 		}
// 	}
	
// 	std::ofstream fil;
// 	if (s_save) fil.open("init.debug");
	
	for (int i = 0; i < g_nPedestrian; i++) {
		index[i] = 1;
		
		vx = g_experiment->getWaypointx(i, 1) - g_experiment->getWaypointx(i, 0);
		vy = g_experiment->getWaypointy(i, 1) - g_experiment->getWaypointy(i, 0);
		
		tmp = sqrt(vx*vx + vy*vy);
		vx /= tmp;
		vy /= tmp;
		
		g_simulation->addState(g_map[i],
			g_experiment->getWaypointx(i, 0),
			g_experiment->getWaypointy(i, 0),
			vx, vy);
		setSimPosition(i,
			g_experiment->getWaypointx(i, 0),
			g_experiment->getWaypointy(i, 0));
		setSimVelocity(i, vx, vy);
		setSimGoal(i,
			g_experiment->getWaypointx(i, 1),
			g_experiment->getWaypointy(i, 1));
		
// 		if (s_save) fil<< g_experiment->getWaypointx(i, 0)<< " "<< g_experiment->getWaypointy(i, 0)<< " "<< g_experiment->getWaypointx(i, 1)<< " "<< g_experiment->getWaypointy(i, 1)<< std::endl;
	}
	
// 	if (s_save) fil.close();
	
// 	std::ofstream file;
// 	if (s_save) file.open("paths.debug");
	
	while (loop && counter < 100000) {
		counter++;
// 		std::cout<< "loop\n";
		loop = false;
		g_simulation->doStep(g_experiment->getTime(1));
		
		for (int i = 0; i < g_nPedestrian; i++) {
			x = getSimPositionxNext(i);
			y = getSimPositionyNext(i);
			vx = getSimVelocityxNext(i);
			vy = getSimVelocityyNext(i);
			
			if (index[i] < g_experiment->getNWaypoint(i)) {
				loop = true;
				
// 				x = getSimPositionxNext(i);
// 				y = getSimPositionyNext(i);
// 				vx = getSimVelocityxNext(i);
// 				vy = getSimVelocityyNext(i);
				
// 				std::cout<< getSimPositionxNext(i)<< " "<< getSimPositionyNext(i)<< std::endl;
				
// 				oldVelx[i] = getSimPositionxNext(i)-getSimPositionx(i, mes);
// 				oldVely[i] = getSimPositionyNext(i)-getSimPositiony(i, mes);
				
				tmp1 = x-g_experiment->getWaypointx(i, index[i]);
				tmp2 = y-g_experiment->getWaypointy(i, index[i]);
				
				tmp = g_experiment->getWaypointSize(i, index[i]);
				if (tmp1*tmp1+tmp2*tmp2 <= tmp*tmp) {
					index[i]++;
					if (index[i] < g_experiment->getNWaypoint(i)) {
						setSimGoal(i,
							g_experiment->getWaypointx(i, index[i]),
							g_experiment->getWaypointy(i, index[i]));
					}
// 					else {
// 						tmp1 = getSimPositionxNext(i) + 10*oldVelx[i];
// 						tmp2 = getSimPositionyNext(i) + 10*oldVely[i];
// 						
// 						setSimGoal(i, tmp1, tmp2);
// 					}
				}
			}
// 			else {
// 				x = getSimPositionx(i, mes)+oldVelx[i];
// 				y = getSimPositiony(i, mes)+oldVely[i];
// 				
// 				vx = oldVelx[i];
// 				vy = oldVely[i];
// 			}
			g_simulation->addState(g_map[i], x, y, vx, vy);
			
// 			if (s_save) file<< x<< " "<< y<< " "<< vx<< " "<< vy<< std::endl;
		}
		mes++;
	}
	
// 	if (s_save) file.close();
	
	delete [] index;
// 	delete [] oldVelx;
// 	delete [] oldVely;
	g_simulation->resetNMeasure();
}

void Record::initSimulation()
{
	g_nPedestrian = g_experiment->getNPedestrian();
	g_nMeasure = g_experiment->getNMeasure();
	
	if (g_simulation->getNObstacle() != g_experiment->getNObstacle()) {
		for (int i = 0; i < g_experiment->getNObstacle(); i++) {
			g_simulation->addObstacle(g_experiment->getObstacleStartx(i),
				g_experiment->getObstacleStarty(i),
				g_experiment->getObstacleEndx(i),
				g_experiment->getObstacleEndy(i));
		}
	}
	
	g_simulation->initSimulation(g_nPedestrian);
	
	g_params = g_simulation->getParam(0, 0);
	g_nParams = g_simulation->getNParam();
	
	if (g_mask) delete [] g_mask;
	if (g_map) delete [] g_map;
	if (g_invmap) delete [] g_invmap;
	g_invmap = 0;
	
	g_mask = new bool[g_nPedestrian];
	g_map = new int[g_nPedestrian];
	
	if (g_experiment->getNSimulated() > 0) {
		for (int i = 0; i < g_nPedestrian; i++)
			g_mask[i] = false;
		
		for (int i = 0; i < g_experiment->getNSimulated(); i++)
			g_mask[g_experiment->getSimulatedIndex(i)] = true;
	}
	else
		for (int i = 0; i < g_nPedestrian; i++)
			g_mask[i] = true;
		
// 	g_mask[0] = true;
// 	g_mask[1] = false;
// 	g_mask[0] = false;
// 	g_mask[1] = false;
// 	g_mask[2] = true;
// 	g_mask[3] = false;
// 	g_mask[4] = true;
// 	g_mask[5] = false;
	saveMask();
	
	p_initSim();
}

void Record::resetSimulation()
{
	g_simulation->reset();
	p_initSim();
}

void Record::setMask(int s_indPedestrian, bool s_simulate)
{
	g_mask[s_indPedestrian] = s_simulate;
}

void Record::saveMask()
{
	g_nPedestrianSimulated = 0;
	
	for (int i = 0; i < g_nPedestrian; i++) {
		if (g_mask[i]) {
			g_map[i] = g_nPedestrianSimulated;
			g_nPedestrianSimulated++;
		}
	}
	
	int count = g_nPedestrianSimulated;
	int count2 = 0;
	
	g_invmap = new int[g_nPedestrianSimulated];
	
	for (int i = 0; i < g_nPedestrian; i++) {
		if (g_mask[i]) {
			g_invmap[count2] = i;
			count2++;
		}
		else {
			g_map[i] = count;
			count++;
		}
	}
}

void Record::computeNext()
{
	g_simulation->doStep(g_experiment->getTime(1)-g_experiment->getTime(0));
}

void Record::computePath(bool s_save)
{
    if (g_experiment->isRealData())
        p_computePathRealData();
	else p_computePathFakeData(s_save);
}

void Record::syncSimulation(float s_time)
{
// 	g_experiment->setTime(s_time);
// 	
// 	for (int i = 0; i < g_nPedestrian; i++) {
// 		setSimPosition(i,
// 			getExpPositionx(i),
// 			getExpPositiony(i));
// 		setSimVelocity(i,
// 			getExpVelocityx(i),
// 			getExpVelocityy(i));
// 	}
}

void Record::sync(float s_time)
{
	g_experiment->setTime(s_time);
	g_simulation->setTime(s_time);
}

void Record::save(const char * s_filename)
{
	FILE * file = fopen(s_filename, "w");
	
	fprintf(file, "%d\n", g_experiment->getNObstacle());
	
	for (int i = 0; i < g_experiment->getNObstacle(); i++) {
		fprintf(file, "%f %f %f %f\n", g_experiment->getObstacleStartx(i)*100.0, g_experiment->getObstacleStarty(i)*100.0, g_experiment->getObstacleEndx(i)*100.0, g_experiment->getObstacleEndy(i)*100.0);
	}
	for (int i = 0; i < getNPedestrian(); i++) {
		for (int j = 0; j < getSimulation()->getNMeasure(); j++) {
			fprintf(file, "%d %d %f %f 175.0\n", i+1, j, getSimPositionx(i, j)*100.0, getSimPositiony(i, j)*100.0);
		}
	}
	
	fclose(file);
}

void Record::saveParams(const char * s_filename)
{
	std::ofstream stream;
	stream.open(s_filename);
	char buffer[16];
	
	stream<< "#--PARAMETERS--\n\n";
	for (int i = 0; i < g_nParams; i++) {
		stream<< "%parameter_"<< i<< std::endl;
		for (int j = 0; j < g_nPedestrianSimulated; j++) {
			sprintf(buffer, "%a", g_params[j*g_nParams+i]);
			stream<< "    "<< buffer<< std::endl;
		}
	}
	
	stream.close();
}

void Record::loadParams(const char * s_filename)
{
	ConfigurationParser p;
	p.parse(s_filename);
	std::stringstream ss;
	bool hexfloat = p.exists("hexfloat");
	
	for (int i = 0; i < g_nParams; i++) {
		ss.str("");
		ss<< "parameter_";
		ss<< i;
		
		for (int j = 0; j < g_nPedestrianSimulated; j++) {
// 			if (hexfloat) sscanf(p.getStringValue(ss.str(), j), "%a", g_params+j*g_nParams+i);
			/*else */g_params[j*g_nParams+i] = p.getFloatValue(ss.str(), j);
		}
	}
	
	g_simulation->reset();
}

void Record::updateSpecific()
{
	if (g_configurablesMap["Save trajectories"].changed && getConfigurableValue("Save trajectories").compare("") != 0) {
		save(g_saveTrajectories.c_str());
		g_saveTrajectories = "";
	}
	if (g_configurablesMap["Save parameters"].changed && getConfigurableValue("Save parameters").compare("") != 0) {
		saveParams(g_saveParameters.c_str());
		g_saveParameters = "";
	}
	if (g_configurablesMap["Load parameters"].changed && getConfigurableValue("Load parameters").compare("") != 0) {
		loadParams(g_loadParameters.c_str());
		g_loadParameters = "";
	}
}

}
