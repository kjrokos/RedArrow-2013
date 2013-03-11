#include "poscntl.h"
#include "math.h"

poscntl::poscntl()
{
	poscntl::k = 0.0;
}

poscntl::poscntl(float x)
{
	poscntl::k = x;
}


float poscntl::FB(int cp, int tp)	//Feedback Function
{
	// k 	: 	gain factor;error scalar
	// cp	:	actual position
	// tp	:	ideal position
	float vo=0.0;
	if((cp-tp)<3 && (cp-tp)>-3) return vo;  // dead band if close return 0 output
	
	// velocity is set to the square root of positional error times gain
	if(cp>tp) vo=-k*sqrt(cp-tp);  
	if(cp<tp) vo= k*sqrt(tp-cp);
	
	// clamp values to max range
	if(vo>1) vo=1.0;
	if(vo<-1.0) vo=-1.0;
	return vo;
}

void poscntl::reset(float x)
{
	poscntl::k = x;
}
void poscntl::reset()
{
	poscntl::k = 0;
}
