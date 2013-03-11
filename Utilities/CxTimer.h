#pragma once
// dmc added Reset to timer methods 201002121932dmc

class CxTimer
{
private:
	// static variable only one set for all timers instances that may be created
	static long m_myLastTm;
	static long m_timeTicks;
	static long m_timeTicksRm;
	// normal variable instance one for every timer created
	long m_myCurTime;

public:
	CxTimer(void); // constructor
	static void Update(void); // gets real time clock tick from FPGA and sets number of ticks for this cycle
	void Reset(void); // resets accumilated time to 0
	long GetTime(void); // gets current accumilated time as ticks (number of .01 seconds)
	double GetTimeSec(void); // scales time into floating point number of seconds
	bool CkTime(bool amon, long tl); // adds time ticks and then checks for if timer has expired 

};
