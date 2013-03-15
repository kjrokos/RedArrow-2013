#include "WPILib.h"
#include "Relay.h"
#include "DigitalInput.h"
#include "TwoStateServoControl.h"
#include "../Utilities/CxTimer.h"

#define STOPPED 0
#define INITIAL_ON 1
#define ON 2
#define REVERSE 3
#define TIMEOUT 4

class FeederControl
{
public:
	FeederControl(uint32_t outputID, uint32_t limitSwitchID);
	~FeederControl();
	
	void Reset();
	
	void Feed();
	void Reverse();
	void Stop();
	
	void ResetNumberOfFeeds();
	
	int GetNumberOfFeeds();
	
	bool Update();
	
	
private:
	Relay *m_feederActuator;
	DigitalInput *m_retractedPosition;
	TwoStateServoControl *m_servo;
	
	int m_numberOfFeeds;
	int m_state;
	
	CxTimer m_timer;
};
