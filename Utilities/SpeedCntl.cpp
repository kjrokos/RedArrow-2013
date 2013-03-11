#include "speedcntl.h"
#include "math.h"



speedcntl::speedcntl(double pki, double iki, double minperiodi)
{
	m_porportionalGain=pki; // porportional error term gain
	m_integralGain=iki; // integral error term gain
	m_minPeriod=minperiodi; // min period for fastest speed
	m_integralTerm=0; // reset integral term to 0;
	m_oldval=0;
}

speedcntl::speedcntl()
{
	m_porportionalGain = 0; // porportional error term gain
	m_integralGain = 0; // integral error term gain
	m_minPeriod = 0; // min period for fastest speed
	m_integralTerm = 0; // reset integral term to 0;
	m_oldval = 0;
}
	
void speedcntl::reset()
{
	//Reset Integral term
	m_integralTerm = 0;
	m_oldval = 0;
}
void speedcntl::reset(double pki, double iki)
{
	m_porportionalGain = pki; // porportional error term gain
	m_integralGain = iki; // integral error term gain
	m_minPeriod = 0; // min period for fastest speed
	m_integralTerm = 0; // reset integral term to 0;
	m_oldval = 0;
}



// implement a simple PI control
// the feedback is period of rotation so have to do some fancy translation
// of throttle setting to period of rotation
// This map of the physical is very non linear and requires the throttle setting
// to be translated using a linear interpolated look up table to period

// ideal time - current time of rotation is error
// two independent gains  m_porportionalGain is porportional gain
// m_integralGain is itegral gain
// clamp itegral term to ouput limits

float speedcntl::controlx(float setPoint, double currentPeriod) //Feedback control
{
	float vo=0.0;
	double acsp=0.0;
	double error=0.0;
	// if the measure period is abnormal
	// too long then it reports infinity and this is required to check for infinity
	if(currentPeriod!=currentPeriod ||(currentPeriod==currentPeriod && ((currentPeriod-currentPeriod)!=0)))  //invalid feedback do not do control
	{
		return setPoint;  // no feed back just output the setpoint
	}

	if(setPoint > .05) // got a set point do control
	{
		
		if(currentPeriod > .4 || currentPeriod < .04) return m_oldval;
		//acsp=m_minPeriod/setPoint;
		acsp=dfuncx(setPoint); // translate setpoint into a period of time for revolutions of wheels
		error=acsp-currentPeriod; // ideal period - current period
		
		//vo=setPoint-m_porportionalGain*error+m_integralTerm;
		vo = setPoint - (m_porportionalGain * error) + m_integralTerm; // produce output with error correction and itegral term
		m_integralTerm -= m_integralGain * error; // update itegral term
		if(m_integralTerm > 1) m_integralTerm = 1; // clamp terms contribution to limits
		if(m_integralTerm < -1) m_integralTerm = -1;
	}
	else // no setpoint  or negative
	{
		m_integralTerm = 0.0;// reset integral term
		vo = 0.0;
	}
	// clamp value to range of control
	if(vo>1)vo=1.0;
	if(vo<0)vo=0.0;
	m_oldval = vo;
	return vo;
	
}
// needed to make a function that translated the natural set point being feed to the ball launcher
// into period of rotation.  This was very non linear and tried several transfer functions to map
// eventually just plotted the graph of measurement and implemented this crude linear interpolation
// between data points.  11 points from 0 to 1
double dfuncx(float sp)
{
	//double dx[]={0,0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0};
	double dx[]={2.5, 0.5, 0.1, 0.064, 0.056, 0.052, 0.050, 0.049, 0.0485, 0.0482, 0.048};
	
	if(sp >= 1.0) return dx[10]; // over range report max value
	if(sp <= 0.0) return dx[0]; // under range report min value
	sp = sp*10; // get rough index
	int ix = (int)sp; // remove remainder for actual index
	float spx = sp-ix; // get remainder isolated
	// calculate portion between points and add to base point the return
	return dx[ix]+spx*(dx[ix+1]-dx[ix]);
}

