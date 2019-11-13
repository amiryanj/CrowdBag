#include "../include/Record3D.h"

namespace craal {

Record3D::Record3D()
{
	name = "Record3D";
}

Record3D::~Record3D()
{}

void Record3D::p_initSim()
{	
	if (g_experiment->isRealData()) {
		for (int i = 0; i < g_nPedestrian; i++) {
			setSimPosition(i,
				getExpPositionx(i, 0),
				getExpPositiony(i, 0),
				getExpPositionz(i, 0));
			setSimVelocity(i,
				getExpVelocityx(i, 0),
				getExpVelocityy(i, 0),
				getExpVelocityz(i, 0));
			setSimGoal(i,
				getExpPositionx(i, g_nMeasure-1),
				getExpPositiony(i, g_nMeasure-1),
				getExpPositionz(i, g_nMeasure-1));
		}
		sync(0);
	}
	else {
		for (int i = 0; i < g_nPedestrian; i++) {
			setSimPosition(i,
				g_experiment->getWaypointx(i, 0),
				g_experiment->getWaypointy(i, 0),
				((Experiment3D*)g_experiment)->getWaypointz(i, 0));
			setSimVelocity(i, 0, 0, 0);
			setSimGoal(i,
				g_experiment->getWaypointx(i, 1),
				g_experiment->getWaypointy(i, 1),
				((Experiment3D*)g_experiment)->getWaypointz(i, 1));
		}
	}
}

void Record3D::p_computePathRealData()
{
	g_simulation->clear();
	
	for (int i = 0; i < g_nPedestrian; i++) {
		((Simulation3D*)g_simulation)->addState(g_map[i],
			getExpPositionx(i, 0),
			getExpPositiony(i, 0),
			getExpPositionz(i, 0),
			getExpVelocityx(i, 0),
			getExpVelocityy(i, 0),
			getExpVelocityz(i, 0));
		setSimPosition(i,
			getExpPositionx(i, 0),
			getExpPositiony(i, 0),
			getExpPositionz(i, 0));
		setSimVelocity(i,
			getExpVelocityx(i, 0),
			getExpVelocityy(i, 0),
			getExpVelocityz(i, 0));
	}
	
	for (int i = 0; i < g_nMeasure-1; i++) {
		g_simulation->doStep(g_experiment->getTime(i+1)-g_experiment->getTime(i));
		
		for (int j = 0; j < g_nPedestrian; j++) {
			if (true || g_mask[j]) {
				((Simulation3D*)g_simulation)->addState(g_map[j],
					getSimPositionxNext(j),
					getSimPositionyNext(j),
					getSimPositionzNext(j),
					getSimVelocityxNext(j),
					getSimVelocityyNext(j),
					getSimVelocityzNext(j));
			
			}
			else {
				((Simulation3D*)g_simulation)->addState(g_map[j],
					getExpPositionx(j, i+1),
					getExpPositiony(j, i+1),
					getExpPositionz(j, i+1),
					getExpVelocityx(j, i+1),
					getExpVelocityy(j, i+1),
					getExpVelocityz(j, i+1));
				setSimPosition(j,
					getExpPositionx(j, i+1),
					getExpPositiony(j, i+1),
					getExpPositionz(j, i+1));
				setSimVelocity(j,
					getExpVelocityx(j, i+1),
					getExpVelocityy(j, i+1),
					getExpVelocityz(j, i+1));
			}
		}
	}
	
	g_simulation->resetNMeasure();
}

void Record3D::p_computePathFakeData(bool s_save)
{
	g_simulation->clear();
	int * index = new int[g_nPedestrian];
// 	float * oldVelx = new float[g_nPedestrian];
// 	float * oldVely = new float[g_nPedestrian];
// 	float * oldVelz = new float[g_nPedestrian];
	bool loop = true;
	float tmp1, tmp2, tmp3, tmp;
	int mes = 0;
	int counter = 0;
	float x, y, z, vx, vy, vz;
	
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
		vz = ((Experiment3D*)g_experiment)->getWaypointz(i, 1) - ((Experiment3D*)g_experiment)->getWaypointz(i, 0);
		
		tmp = sqrt(vx*vx + vy*vy + vz*vz);
		vx /= tmp;
		vy /= tmp;
		vz /= tmp;
		
		((Simulation3D*)g_simulation)->addState(g_map[i],
			g_experiment->getWaypointx(i, 0),
			g_experiment->getWaypointy(i, 0),
			((Experiment3D*)g_experiment)->getWaypointz(i, 0),
			vx, vy, vz);
		setSimPosition(i,
			g_experiment->getWaypointx(i, 0),
			g_experiment->getWaypointy(i, 0),
			((Experiment3D*)g_experiment)->getWaypointz(i, 0));
		setSimVelocity(i, vx, vy, vz);
		setSimGoal(i,
			g_experiment->getWaypointx(i, 1),
			g_experiment->getWaypointy(i, 1),
			((Experiment3D*)g_experiment)->getWaypointz(i, 1));
		
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
			z = getSimPositionzNext(i);
			vx = getSimVelocityxNext(i);
			vy = getSimVelocityyNext(i);
			vz = getSimVelocityzNext(i);
			
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
				tmp3 = z-((Experiment3D*)g_experiment)->getWaypointz(i, index[i]);
				
				tmp = g_experiment->getWaypointSize(i, index[i]);
				if (tmp1*tmp1+tmp2*tmp2+tmp3*tmp3 <= tmp*tmp) {
					index[i]++;
					if (index[i] < g_experiment->getNWaypoint(i)) {
						setSimGoal(i,
							g_experiment->getWaypointx(i, index[i]),
							g_experiment->getWaypointy(i, index[i]),
							((Experiment3D*)g_experiment)->getWaypointz(i, index[i]));
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
			((Simulation3D*)g_simulation)->addState(g_map[i], x, y, z, vx, vy, vz);
			
// 			if (s_save) file<< x<< " "<< y<< " "<< vx<< " "<< vy<< std::endl;
		}
		mes++;
	}
	
// 	if (s_save) file.close();
	
	delete [] index;
// 	delete [] oldVelx;
// 	delete [] oldVely;
// 	delete [] oldVelz;
	g_simulation->resetNMeasure();
}

void Record3D::save(const char * s_filename)
{
	FILE * file = fopen(s_filename, "w");
	
	fprintf(file, "%d\n", g_experiment->getNObstacle());
	
	for (int i = 0; i < g_experiment->getNObstacle(); i++) {
		fprintf(file, "%f %f %f %f\n", g_experiment->getObstacleStartx(i)*100.0, g_experiment->getObstacleStarty(i)*100.0, g_experiment->getObstacleEndx(i)*100.0, g_experiment->getObstacleEndy(i)*100.0);
	}
	for (int i = 0; i < getNPedestrian(); i++) {
		for (int j = 0; j < getSimulation()->getNMeasure(); j++) {
			fprintf(file, "%d %d %f %f %f\n", i+1, j,
				getSimPositionx(i, j)*100.0,
				getSimPositiony(i, j)*100.0,
				getSimPositionz(i, j)*100.0);
		}
	}
	
	fclose(file);
}

}
