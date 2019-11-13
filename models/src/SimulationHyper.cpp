#include "SimulationHyper.h"
#ifdef ORACLE
namespace craal {

SimulationHyper::SimulationHyper() :
	g_oracle(0)
{
	addParam("Speed", 1.6, PDF(1, 2, PDF::NORMAL, 1.5, 0.5));
	addParam("Threshold", 0.2, PDF(0, 1, PDF::NORMAL, 0.5, 0.75));
	addParam("ChangeAmountSpeed", 0.2, PDF(0, 10, PDF::NORMAL, 5, 5));
	addParam("ChangeAmountOri", 0.2, PDF(0, 10, PDF::NORMAL, 5, 5));
	addParam("SpeedAccel", 0.1, PDF(0, 0.5, PDF::NORMAL, 0.25, 0.25));
	addParam("OriAccel", 0.1, PDF(0, 0.5, PDF::NORMAL, 0.25, 0.25));
	addParam("SpeedUnx", 1.0, PDF(0, 5, PDF::NORMAL, 2.5, 2.5));
	addParam("SpeedUny", 1.0, PDF(0, 5, PDF::NORMAL, 2.5, 2.5));
	
	name = "HyperModel";
// 	g_first = true;
}

SimulationHyper::~SimulationHyper()
{
	if (g_oracle) delete g_oracle;
}

void SimulationHyper::addObstacleCoords(float s_startx, float s_starty, float s_endx, float s_endy)
{}

void SimulationHyper::init()
{
	if (g_oracle) delete g_oracle;
// 	if (g_first) {
// 		delete [] g_params;
// 	
// 		float tmp = OHSHIT*OHSHIT*OHSHIT;
// 		tmp /= ((float)g_nPedestrian);
// 		tmp += 1.0;
// 		int tmpp = tmp;
// 		
// 		std::stringstream ss;
// 		
// 		for (int i = 0; i < tmpp; i++) {
// 			ss.str("");
// 			ss<< "ht_";
// 			ss<< i;
// 			addParam(ss.str(), 0.5, PDF(0, 1, PDF::NORMAL, 0.5, 0.5));
// 		}
// 		
// 		g_params = new float[g_nPedestrian*g_nbParams];
// 		g_first = false;
// 	}
	
	g_oracle = new hyperModel::Oracle();
	g_oracle->addDistortion(hyperModel::Distortion::REFERENTIAL);
	g_oracle->addDistortion(hyperModel::Distortion::SPEED);
	g_oracle->addDistortion(hyperModel::Distortion::SPEED_UNCERTAINTY);
	g_oracle->addDistortion(hyperModel::Distortion::SIZE);
	g_oracle->precision = 20;
	
	int OHSHIT = 15;
	
	g_oracle->length = OHSHIT;
	g_oracle->width = OHSHIT;
	g_oracle->height = OHSHIT;
	g_oracle->init();
	
	float x, y, z;
	float r, R;
	float tmp;
	for (int i = 0; i < OHSHIT; i++) {
		x = i;
		x -= ((float)(OHSHIT-1))/2.0;
		
		for (int j = 0; j < OHSHIT; j++) {
			y = j;
			y -= ((float)(OHSHIT-1))/2.0;
			
			r = sqrt(x*x+y*y);
			
			for (int k = 0; k < OHSHIT-1; k++) {
				z = k;
				
				R = (((float)(OHSHIT-1))/4.0)*z/((float)(OHSHIT-2));
				if (r > R)
					tmp = R-r;
				else
					tmp = 1.0-r/R;
				
				tmp = MIN(MAX(tmp, -1.0), 1.0);
				tmp = (tmp+1.0)/2.0;
				
				g_oracle->hypertexture(i, j, k).prob = tmp;
			}
			g_oracle->hypertexture(i, j, OHSHIT-1).prob = 0;
		}
	}
	g_oracle->hypertexture.computeGradients();
	
// 	for (int i = 0; i < OHSHIT*OHSHIT*OHSHIT; i++)
// 		g_oracle->hypertexture[i] = hyperModel::HyperTexture::Probability(0, 0, 0, 0);
	
	for (int i = 0; i < g_nPedestrian; i++) {
		g_oracle->addAgent();
		g_oracle->agent(i).anticipationTime = 5;
		g_oracle->agent(i).anticipationTimeInverted = 1.0/5.0;
		g_oracle->agent(i).threshold = 0.2;
		g_oracle->agent(i).changeAmountSpeed = 0.2;
		g_oracle->agent(i).changeAmountOri = 0.2;
		g_oracle->agent(i).comfortSpeed = 1.6;
		g_oracle->agent(i).comfortAccel = 0.1;
		g_oracle->agent(i).goalOriAccel = 0.1;
		g_oracle->agent(i).speedUncertaintyx = 1;
		g_oracle->agent(i).speedUncertaintyy = 1;
		g_oracle->agent(i).speedUncertaintyInvertedx = 1;
		g_oracle->agent(i).speedUncertaintyInvertedy = 1;
	}
}

void SimulationHyper::reset()
{
	hyperModel::HyperTexture::Probability p;
	int count = 0;
	for (int i = 0; i < g_nPedestrian; i++) {
		g_oracle->agent(i).comfortSpeed = g_params[i*g_nbParams+0];
		g_oracle->agent(i).threshold = g_params[i*g_nbParams+1];
		g_oracle->agent(i).changeAmountSpeed = g_params[i*g_nbParams+2];
		g_oracle->agent(i).changeAmountOri = g_params[i*g_nbParams+3];
		g_oracle->agent(i).comfortAccel = g_params[i*g_nbParams+4];
		g_oracle->agent(i).goalOriAccel = g_params[i*g_nbParams+5];
		g_oracle->agent(i).speedUncertaintyx = g_params[i*g_nbParams+6];
		g_oracle->agent(i).speedUncertaintyy = g_params[i*g_nbParams+7];
		g_oracle->agent(i).speedUncertaintyInvertedx = 1.0/g_oracle->agent(i).speedUncertaintyx;
		g_oracle->agent(i).speedUncertaintyInvertedy = 1.0/g_oracle->agent(i).speedUncertaintyy;
		
// 		for (int j = 8; j < g_nbParams; j++) {
// 			if (count < OHSHIT*OHSHIT*OHSHIT) {
// 				p.prob = g_params[i*g_nbParams+j];
// 				g_oracle->hypertexture[count] = p;
// 			}
// 			count++;
// 		}
	}
// 	g_oracle->hypertexture.computeGradients();
	g_oracle->initSteps();
}

void SimulationHyper::setPosition(int s_indPedestrian, float s_x, float s_y)
{
	g_oracle->agent(s_indPedestrian).x = s_x;
	g_oracle->agent(s_indPedestrian).y = s_y;
}

void SimulationHyper::setVelocity(int s_indPedestrian, float s_x, float s_y)
{
	if (s_x == 0 && s_y == 0) {
		g_oracle->agent(s_indPedestrian).speed = 0;
		g_oracle->agent(s_indPedestrian).orientation = 0;
		g_oracle->agent(s_indPedestrian).update();
		return;
	}
	
	float tmp = sqrt(s_x*s_x+s_y*s_y);
	g_oracle->agent(s_indPedestrian).speed = tmp;
	if (s_y < 0)
		g_oracle->agent(s_indPedestrian).orientation = -acos(s_x/tmp);
	else
		g_oracle->agent(s_indPedestrian).orientation = acos(s_x/tmp);
	
	g_oracle->agent(s_indPedestrian).update();
}

void SimulationHyper::setGoal(int s_indPedestrian, float s_x, float s_y)
{
	g_oracle->agent(s_indPedestrian).goalx = s_x;
	g_oracle->agent(s_indPedestrian).goaly = s_y;
}

void SimulationHyper::doStep(float s_dt)
{
	g_oracle->doStep(s_dt);
	
	for (int i = 0; i < g_nPedestrian; i++) {
		setNextState(i,
					g_oracle->agent(i).x,
					g_oracle->agent(i).y,
					g_oracle->agent(i).cosOri*g_oracle->agent(i).speed,
					g_oracle->agent(i).sinOri*g_oracle->agent(i).speed);
	}
}

}
#endif
