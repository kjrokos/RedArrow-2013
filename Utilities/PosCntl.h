#pragma once


// simple positional feedback control based on velocity is square root of positional error
// a single gain factor
// 
class poscntl
{
private:
	int targetp;  //spare not used
	float k;
	
public:
	//Constructors
	poscntl();
	poscntl(float k);
	
	
	float FB(int cp, int tp);	//Feedback Function
	
	void reset(float k);
	void reset();
	
	//Return Functions
};
