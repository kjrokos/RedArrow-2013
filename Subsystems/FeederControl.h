#include "WPILib.h"
#include "Relay.h"
#include "DigitalInput.h"
#include "TwoStateServoControl.h"

#define STOPPED 0
#define INITIAL_ON 1
#define ON 2
#define REVERSE 3

class FeederControl
{
public:
	FeederControl(uint32_t outputID, uint32_t servoID, uint32_t limitSwitchID);
	~FeederControl();
	
	void Reset();
	
	void Feed();
	void Reverse();
	
	void UnjammerUp();
	void UnjammerDown();
	
	void ResetNumberOfFeeds();
	
	int GetNumberOfFeeds();
	
	bool Update();
	
	
private:
	Relay *m_feederActuator;
	DigitalInput *m_retractedPosition;
	TwoStateServoControl *m_servo;
	
	int m_numberOfFeeds;
	int m_state;
	
	bool m_manualControl;
	
};
