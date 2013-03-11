//
//  PotentiometerControl.h
//  First 2013
//
//  Created by Kyle Rokos on 1/22/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#ifndef __First_2013__PotentiometerControl__
#define __First_2013__PotentiometerControl__

#include <iostream>
#include "Talon.h"
#include "DigitalInput.h"
#include "AnalogChannel.h"
#include "../Utilities/PosCntl.h"

class PotentiometerControl
{
public:
    PotentiometerControl(uint32_t talonID, uint32_t analogChannelID, uint16_t lowerLimit, uint16_t upperLimit, float k = .03);
    ~PotentiometerControl();
    
    void Reset();
    
    // return true when current commands are finished
    bool Update();
    
    void ManualControl(float stickInput, float sensitivity=10);
    
    void SetPercent(float setPoint);
    void SetPosition(uint16_t setPoint);
    //void SetAngle(float setPoint);    // need additional information to compute this
    
    float GetPercent();
    uint16_t GetPosition();
    //float GetAngle();                 // need additional information to compute this
    
private:
    
    // ensures m_setPosition is within specified lower and upper limits
    void assureBounds();
    
    //turret elevation motor
    Talon *m_pElevator;
    
	//Limits
	uint16_t m_lowerLimit;		//Stop for Turret Elevation
	uint16_t m_upperLimit;		//Stop for Turret Elevation

    //Encoder
    AnalogChannel *m_pElevation;
	
    //Position Control
    poscntl *m_pPotentiometerControl;

    uint16_t m_setPosition;
    
};


#endif /* defined(__First_2013__PotentiometerControl__) */
