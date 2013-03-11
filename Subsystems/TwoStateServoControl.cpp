//
//  TwoStateServoControl.cpp
//  First 2013
//
//  Created by Kyle Rokos on 1/20/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#include "TwoStateServoControl.h"


TwoStateServoControl::TwoStateServoControl(uint32_t outputID, float downPosition, float upPosition)
    : m_raised(false),
    m_servo(new Servo(outputID)),
    m_downPosition(downPosition),
    m_upPosition(upPosition)
{
    Lower();
}

TwoStateServoControl::~TwoStateServoControl()
{
	delete m_servo;
}

void TwoStateServoControl::Reset()
{
	Lower();
}

void TwoStateServoControl::Raise()
{
    m_raised = true;
};

void TwoStateServoControl::Lower()
{
    m_raised = false;
};

void TwoStateServoControl::SetState(bool up)
{
    if(up)
        Raise();
    else
        Lower();
};

bool TwoStateServoControl::IsRaised()
{
    return m_raised;
};

bool TwoStateServoControl::Update()
{
	if(IsRaised())
		m_servo->Set(m_upPosition);
	else
		m_servo->Set(m_downPosition);

	return true;
};
