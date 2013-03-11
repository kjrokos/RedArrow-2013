//
//  TwoStateServoControl.h
//  First 2013
//
//  Created by Kyle Rokos on 1/20/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#ifndef __First_2013__TwoStateServoControl__
#define __First_2013__TwoStateServoControl__

#include <iostream>
#include "WPILib.h"

class TwoStateServoControl
{
public:
    TwoStateServoControl(uint32_t outputID, float downPosition, float upPosition);
    ~TwoStateServoControl();
    
    void Reset();
    
    void Raise();
    void Lower();
    
    void SetState(bool up);
    bool IsRaised();
    
    // return true when current commands are finished
    bool Update();
    
private:
    bool m_raised;
    Servo *m_servo;
    float m_downPosition;
    float m_upPosition;
};

#endif /* defined(__First_2013__TwoStateServoControl__) */
