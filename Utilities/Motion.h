#pragma once

class Motion
{
private:
	int m_encoderIP;
	double m_initTime;
	double m_runTime;
	float m_k;
	bool m_active;
	float m_totalMove;
	
public:
	//Constructors
	Motion();
	Motion(float k);
	
	double P345(double t);					//Position 345 Polynomial
	double V345(double t);					//Velocity 345 Polynomial
	
	float FB(float k, float pa, float ps);	//Feedback Function
	
	float AdjustVelocity(int currentCount, double time);
	void Reset(int count, double time, float move, double runTime);
	void Reset();
	
	//Return Functions
	float Getk();
	double GetRunTime();
	bool GetActive();
};
