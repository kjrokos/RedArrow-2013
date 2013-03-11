#ifndef __First_2013__ArmControl__
#define __First_2013__ArmControl__

#include "Talon.h"
#include "DigitalInput.h"
#define ARM_UP true
#define ARM_DOWN false

class ArmControl
{
public:
	ArmControl(uint32_t outputChannel, uint32_t lowerLimitSwitchChannel, uint32_t upperLimitSwitchChannel);
	~ArmControl();
	
	void Reset();
	
	void ArmUp();
	void ArmDown();
	void ManualControl(float stick);
	
	bool IsArmUp();
	bool IsArmDown();
	
	bool Update();
	
private:
	Talon *m_armActuator;
	DigitalInput *m_lowerPosition;
	DigitalInput *m_upperPosition;
	
	bool m_setPosition;
	bool m_on;
	bool m_manualControl;
	float m_manualControlSpeed;
};

#endif
