#include "CxTimer.h"
#include "time.h"
#include "Utility.h"
// dmc added Reset to timer methods 201002121932dmc


	CxTimer::CxTimer(void)
	{
		// may remove these three resets of static variables DMC
		// all timers must be created prior to using any timer
		m_myLastTm=0;
		m_timeTicks=0;
		m_timeTicksRm=0;
		
		// each instance class variable
		m_myCurTime=0;
	}
	// used only once per periodict routine and on only one timer to set the ticks
	// for all timers used in that periodict routine
	void CxTimer::Update(void)
	{
		long sf=10000; // take it from microsec to millisec
		long maxsf=0x7fffffff; // mask off any sign bit because of actual unsigned count
		long curt=GetFPGATime(); // gets number of microsecond ticks in the current counter
		
		curt=curt&maxsf; // fix for signed vs unsigned
		
		
		if(curt>=m_myLastTm) 
			m_timeTicks=curt-m_myLastTm;  // not roll over just normal
		else
			m_timeTicks=maxsf-(m_myLastTm-curt)+1; // handle roll over case
		// the roll over case is designed to keep with math limits and use normal oprands
		// it would be simpler to do a two complement but with the uncertainty about the 
		// compilers actual methods a pure operator methods was used. 
		
		// because of the scaling a simple integer division will always leave some
		// number of ticks out and overtime will cause the timer to lose accuracy
		// keeping the sum of ticks from the remainder allows for long term accuracy
		// 
		m_timeTicksRm+=(m_timeTicks%sf); // add any remainder to remainder
		// Now get the whole number part of the ticks
		m_timeTicks=m_timeTicks/sf;  // get millisec from microsec clock
		// if enough fractional ticks have accumilated increase the tick count by 1
		while (m_timeTicksRm>sf) {m_timeTicks++;m_timeTicksRm-=sf;} // add odd time 
		// keep this count to use as the last count in then next call
		m_myLastTm=curt;
	}
	long CxTimer::GetTime() // returns the number of accumilated .01 time ticks
	{
		return m_myCurTime;
	}
	void CxTimer::Reset() // sets accumilated time to zero
	{
		m_myCurTime=0;
	}
	
	double CxTimer::GetTimeSec() // returns a scaled time in seconds for the accumilated time
	{
		return (double)m_myCurTime/100.0 ;
	}

	bool CxTimer::CkTime(bool amon, long tl) // add ticks and check for expired timer
	{
		if(!amon) // if not on reset time and return false
		{
			m_myCurTime=0;
			return false;
		}
		m_myCurTime+=m_timeTicks; // add time ticks created by the update use
		if(m_myCurTime>=tl) // if enought time has accumilated return true and clamp value to preset
		{
			m_myCurTime=tl;
			return true;
		}
	return false;
	};
	
	// declared outside of scope as global variables
	// only one set of these three for all timers created
	// updated by use of Update
	
	long CxTimer::m_myLastTm=0;
	long CxTimer::m_timeTicks=0;
	long CxTimer::m_timeTicksRm=0;

