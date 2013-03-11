#include "FeederControl.h"

FeederControl::FeederControl(uint32_t outputID, uint32_t servoID, uint32_t limitSwitchID)
:m_feederActuator(new Relay(outputID, Relay::kBothDirections)),
 m_retractedPosition(new DigitalInput(limitSwitchID)),
 //m_servo(new TwoStateServoControl(servoID ,0,.9)),
 m_numberOfFeeds(0),
 m_state(STOPPED),
 m_manualControl(false)
{
	m_feederActuator->Set(Relay::kOff);
	//m_servo->Raise();
}

FeederControl::~FeederControl()
{
	delete m_feederActuator;
	delete m_retractedPosition;
	//delete m_servo;
}

void FeederControl::Reset()
{
	m_numberOfFeeds = 0;
	this->Reverse(); // reverse if not in stopped position

	//m_feederActuator->Set(Relay::kOff);
	//m_state = STOPPED;
	
	//m_servo->Raise();
	m_manualControl = false;
}

void FeederControl::Feed()
{
	if(m_state == STOPPED)
	{
		m_state = INITIAL_ON;
		//m_servo->Lower();
	}
		
}

void FeederControl::Reverse()
{
	if(m_state != STOPPED)
		m_state = REVERSE;
}

void FeederControl::UnjammerUp()
{
	//m_servo->Raise();
	m_manualControl = true;
}

void FeederControl::UnjammerDown()
{
	//m_servo->Lower();
	m_manualControl = true;
}

void FeederControl::ResetNumberOfFeeds()
{
	m_numberOfFeeds = 0;
}

int FeederControl::GetNumberOfFeeds()
{
	return m_numberOfFeeds;
}

bool FeederControl::Update()
{
	//printf("in FeederControl::Update state = %d\n", m_state);
	//m_servo->Update();
	
	if(m_state == STOPPED)
	{
		//printf("m_state == STOPPED\n");
		m_feederActuator->Set(Relay::kOff);
		//if(m_manualControl == false)
		//	m_servo->Raise();
		m_manualControl = false;
		return true;
	}
	
	bool isRetracted = m_retractedPosition->Get();
	
	if(isRetracted && m_state == INITIAL_ON)
	{
		//printf("m_state == INITIAL_ON && retracted\n");
		m_feederActuator->Set(Relay::kForward);
		return false;
	}
	
	if(!isRetracted && m_state == INITIAL_ON)
	{
		//printf("m_state == INITIAL_ON && not retracted\n");
		m_feederActuator->Set(Relay::kForward);
		m_state = ON;
		return false;
	}
	
	if(!isRetracted && m_state == ON)
	{
		//printf("m_state == ON && not retracted\n");
		m_state = ON;
		return false;
	}
	
	if(isRetracted && m_state == ON)
	{
		//printf("m_state == ON && retracted");
		m_feederActuator->Set(Relay::kOff);
		m_state = STOPPED;
		m_numberOfFeeds+=1;
		return true;
	}
	
	if(!isRetracted && m_state == REVERSE)
	{
		m_feederActuator->Set(Relay::kReverse);
		return false;
	}
	if(isRetracted && m_state == REVERSE)
	{
		m_feederActuator->Set(Relay::kOff);
		m_state = STOPPED;
		return true;
	}
	
	//We Shouldn't Get Here
	m_feederActuator->Set(Relay::kOff);
	m_state = STOPPED;
	return true;
}
