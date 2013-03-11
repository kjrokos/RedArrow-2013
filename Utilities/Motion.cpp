#include "Motion.h"
#include "math.h"

Motion::Motion()
{
	Motion::m_k = 0.0;
}

Motion::Motion(float x)
{
	Motion::m_k = x;
}

double Motion::P345(double t)
{
	double t2=t*t;
	double t3=t2*t;
	return (10*t3)-(15*t2*t2)+(6*t2*t3);
}

double Motion::V345(double t)
{
	double t2=t*t;
	double t3=t2*t;
	return (30*t2)-(60*t3)+(30*t2*t2);
}

float Motion::FB(float m_k, float pa, float ps)	//Feedbacm_k Algorithm
{
	// m_k 	: 	gain factor;error scalar
	// pa	:	actual position
	// ps	:	ideal position
	float v=0;
	if(pa>ps) v=-m_k*sqrt(pa-ps);
	if(pa<ps) v= m_k*sqrt(ps-pa);
	if(v<-1) v=-1.0;
	if(v>1)  v=1.0;
	return v;
}

float Motion::AdjustVelocity(int currentCount, double time)
{
	float ReLT;
	if(time-Motion::m_initTime>Motion::m_runTime) {ReLT=1;m_active=false;} else ReLT=((time-Motion::m_initTime)/Motion::m_runTime);
	float currentP = Motion::m_totalMove*P345(ReLT) + Motion::m_encoderIP;
	if (m_active)return FB( Motion::m_k, currentP, currentCount); else return 0;
}

void Motion::Reset(int count, double time, float move, double m_runTime)
{
	Motion::m_encoderIP=count;
	Motion::m_initTime=time;
	Motion::m_runTime=m_runTime;
	Motion::m_totalMove=move;
	Motion::m_active=true;
}

void Motion::Reset()
{
	Motion::m_encoderIP=0;
	Motion::m_initTime=0.0;
	Motion::m_runTime=0.0;
	Motion::m_totalMove=0.0;
	Motion::m_active=false;
}

//Get Functions
float Motion::Getk(){return Motion::m_k;}
double Motion::GetRunTime(){return Motion::m_runTime;}
bool Motion::GetActive(){return Motion::m_active;}
