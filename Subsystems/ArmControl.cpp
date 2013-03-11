#include "ArmControl.h"

ArmControl::ArmControl(uint32_t outputChannel, uint32_t lowerLimitSwitchChannel, uint32_t upperLimitSwitchChannel)
: m_armActuator(new Talon(outputChannel)),
  m_lowerPosition(new DigitalInput(lowerLimitSwitchChannel)),
  m_upperPosition(new DigitalInput(upperLimitSwitchChannel)),
  m_setPosition(ARM_DOWN),
  m_on(false),
  m_manualControl(false)
{
	
}

ArmControl::~ArmControl()
{
	delete m_armActuator;
	delete m_lowerPosition;
	delete m_upperPosition;
}

void ArmControl::Reset()
{
	m_setPosition = ARM_DOWN;
	m_on = false;
	m_manualControl = false;
}

void ArmControl::ArmUp()
{
	m_setPosition = ARM_UP;
	m_manualControl = false;
	m_on = true;
}
void ArmControl::ArmDown()
{
	m_setPosition = ARM_DOWN;
	m_manualControl = false;
	m_on = true;
}

void ArmControl::ManualControl(float stick)
{
	if(stick < 0)
		m_setPosition = ARM_DOWN;
	if(stick > 0)
		m_setPosition = ARM_UP;
	
	m_on = true;
	m_manualControl = true;
	m_manualControlSpeed = stick;
}
bool ArmControl::IsArmUp()
{
	return m_upperPosition->Get();
}


bool ArmControl::IsArmDown()
{
	return m_lowerPosition->Get();
}

	
bool ArmControl::Update()
{
	if(m_on == false)
	{
		m_armActuator->Set(0);
		return true;
	}
	if(m_setPosition == ARM_UP && !IsArmUp())
	{
		m_armActuator->Set(0);
		m_on = false;
		return true;
	}
	
	if(m_setPosition == ARM_DOWN && !IsArmDown())
	{
		m_armActuator->Set(0);
		m_on = false;
		return true;
	}
	
	if(m_manualControl)
	{
		m_armActuator->Set(m_manualControlSpeed);
		return true;
	}
	else
	{
		if(m_setPosition == ARM_UP)
			m_armActuator->Set(.5);
		
		if(m_setPosition == ARM_DOWN)
			m_armActuator->Set(-.5);
		
		return false;
	}
}
