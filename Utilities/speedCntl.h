#pragma once

double dfuncx(float sp);
class speedcntl
{
private:
	double m_porportionalGain;
	double m_integralGain;
	double m_integralTerm;
	double m_minPeriod;
	double m_oldval;
	
public:
	//Constructors
	speedcntl();
	speedcntl(double pki, double iki, double minperiodi);
	void reset();
	void reset(double pki, double iki);
	
	//Return Functions
	// feed it the current set point from throttle or other source sp
	// feed current period as cp
	float controlx(float sp, double cp);
	
};

// TODO: add negative speed controls (if set to accept them)
