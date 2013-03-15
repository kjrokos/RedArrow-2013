#include "FeederControl.h"

FeederControl::FeederControl(uint32_t outputID, uint32_t limitSwitchID)
:m_feederActuator(new Relay(outputID, Relay::kBothDirections)),
 m_retractedPosition(new DigitalInput(limitSwitchID)),
 m_numberOfFeeds(0),
 m_state(STOPPED)
{
	m_timer.Reset();
	m_feederActuator->Set(Relay::kOff);
}

FeederControl::~FeederControl()
{
	delete m_feederActuator;
	delete m_retractedPosition;
}

void FeederControl::Reset()
{
	m_timer.Reset();
	m_numberOfFeeds = 0;
	this->Reverse(); // reverse if not in stopped position
}

void FeederControl::Feed()
{
	if(m_state == STOPPED || m_state == TIMEOUT)
	{
		m_timer.Reset();
		m_state = INITIAL_ON;
		//m_servo->Lower();
	}
		
}

void FeederControl::Reverse()
{
	if(m_state != STOPPED)
	{
		m_timer.Reset();
		m_state = REVERSE;
	}
}

void FeederControl::Stop()
{
	m_state = STOPPED;
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
	
	if(m_timer.CkTime(true, 600) && m_state != STOPPED)
	{
		m_state = TIMEOUT;
		m_timer.Reset();
	}
	
	if(m_state == STOPPED || m_state == TIMEOUT)
	{
		SmartDashboard::PutString("Feeder State", "Stopped");
		m_feederActuator->Set(Relay::kOff);
		m_timer.Reset();
		return true;
	}
	
	bool isRetracted = m_retractedPosition->Get();
	
	if(isRetracted && m_state == INITIAL_ON)
	{
		SmartDashboard::PutString("Feeder State", "Initial On");
		m_feederActuator->Set(Relay::kForward);
		return false;
	}
	
	if(!isRetracted && m_state == INITIAL_ON)
	{
		SmartDashboard::PutString("Feeder State", "Initial On");
		m_feederActuator->Set(Relay::kForward);
		m_state = ON;
		return false;
	}
	
	if(!isRetracted && m_state == ON)
	{
		SmartDashboard::PutString("Feeder State", "On");
		m_state = ON;
		return false;
	}
	
	if(isRetracted && m_state == ON)
	{
		SmartDashboard::PutString("Feeder State", "Stopping");
		m_feederActuator->Set(Relay::kOff);
		m_state = STOPPED;
		m_numberOfFeeds+=1;
		return true;
	}
	
	if(!isRetracted && m_state == REVERSE)
	{
		SmartDashboard::PutString("Feeder State", "Reverse");
		m_feederActuator->Set(Relay::kReverse);
		return false;
	}
	if(isRetracted && m_state == REVERSE)
	{
		SmartDashboard::PutString("Feeder State", "Stopping");
		m_feederActuator->Set(Relay::kOff);
		m_state = STOPPED;
		return true;
	}
	
	//We Shouldn't Get Here
	SmartDashboard::PutString("Feeder State", "Unknown State (Stopped)");
	m_feederActuator->Set(Relay::kOff);
	m_state = STOPPED;
	return true;
}
