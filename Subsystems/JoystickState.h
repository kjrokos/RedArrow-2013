//
//  JoystickCollection.h
//  First 2013
//
//  Created by Kyle Rokos on 1/29/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#ifndef __First_2013__JoystickState__
#define __First_2013__JoystickState__

#include <iostream>
#include <vector>
#include "Joystick.h"
#include "DriverStation.h"


class JoystickState : public Joystick
{
public:
    JoystickState(uint32_t port);
    ~JoystickState();
    
    void Reset();
    void Update();
    
    bool Pressed(uint32_t buttonID);
    
    bool Clicked(uint32_t buttonID);
    bool Held(uint32_t buttonID);
    bool Released(uint32_t buttonID);
    
    float GetXAxis();
    float GetYAxis();
    float GetZAxis();
    
private:
    bool GetCurrent(uint32_t buttonID);
    bool GetLast(uint32_t buttonID);

private:
    std::vector<float> m_axisValues;
    uint32_t m_buttonStates;
    uint32_t m_lastButtonStates;
    
    DriverStation *m_ds;
    uint32_t m_port;
};

#endif /* defined(__First_2013__JoystickState__) */
