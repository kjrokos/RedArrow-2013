//
//  JoystickState.cpp
//  First 2013
//
//  Created by Kyle Rokos on 1/29/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#include "JoystickState.h"


JoystickState::JoystickState(uint32_t port)
    :Joystick(port),
    m_axisValues(6, 0.f),
    m_buttonStates(0),
    m_lastButtonStates(0),
    m_ds(NULL),
    m_port(port)

{
    m_ds = DriverStation::GetInstance();
}

JoystickState::~JoystickState()
{
    // we don't own the driver station; don't delete it!
}

void JoystickState::Reset()
{
    for(int axis=0; axis<6; axis++)
        m_axisValues[axis] = 0.f;
    m_lastButtonStates = 0;
    m_buttonStates = 0;
}

void JoystickState::Update()
{
    for(int axis=0; axis<6; axis++)
        m_axisValues[axis] = GetRawAxis(axis+1);
    
    m_lastButtonStates = m_buttonStates;
    m_buttonStates = m_ds->GetStickButtons(m_port);
}


bool JoystickState::Pressed(uint32_t buttonID)
{
    return GetCurrent(buttonID);
}

bool JoystickState::Clicked(uint32_t buttonID)
{
    return !GetLast(buttonID) && GetCurrent(buttonID);
}
bool JoystickState::Held(uint32_t buttonID)
{
    return GetLast(buttonID) && GetCurrent(buttonID);
}
bool JoystickState::Released(uint32_t buttonID)
{
    return GetLast(buttonID) && !GetCurrent(buttonID);
}


float JoystickState::GetXAxis()
{
    return m_axisValues[0];
}
float JoystickState::GetYAxis()
{
    return m_axisValues[1];
}
float JoystickState::GetZAxis()
{
    return m_axisValues[2];
}


bool JoystickState::GetCurrent(uint32_t buttonID)
{
    return ((0x1 << (buttonID-1)) & m_buttonStates) != 0;
}

bool JoystickState::GetLast(uint32_t buttonID)
{
    return ((0x1 << (buttonID-1)) & m_lastButtonStates) != 0;
}
