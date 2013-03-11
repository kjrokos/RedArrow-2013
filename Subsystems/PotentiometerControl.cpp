//
//  PotentiometerControl.cpp
//  First 2013
//
//  Created by Kyle Rokos on 1/22/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#include "PotentiometerControl.h"
#include <math.h>

PotentiometerControl::PotentiometerControl(uint32_t talonID, uint32_t analogChannelID, uint16_t lowerLimit, uint16_t upperLimit, float k)
: m_pElevator(new Talon(talonID)),
  m_lowerLimit(lowerLimit),
  m_upperLimit(upperLimit),
  m_pElevation(new AnalogChannel(analogChannelID)),
  m_pPotentiometerControl(new poscntl(k)),
  m_setPosition(0)
{
	m_pElevator->Set(0);
    m_setPosition = m_pElevation->GetValue();
    //printf("PotentiometerControl::Constructor: m_setPosition: %d\n", m_setPosition);
}

PotentiometerControl::~PotentiometerControl()
{
    delete m_pElevator;
    delete m_pElevation;
    delete m_pPotentiometerControl;
}

void PotentiometerControl::Reset()
{
	m_pElevator->Set(0);
	m_setPosition=m_pElevation->GetValue();
}

bool PotentiometerControl::Update()
{
    float elevation = 0.0;
    uint16_t currentPosition = m_pElevation->GetValue();
    
    if(currentPosition == 0 || currentPosition == 1 || currentPosition == 65535)
    {
    	//printf("Invalid potentiometer reading\n");
        m_pElevator->Set(0);
    	return true;
    }
    
    if(currentPosition == m_setPosition)
    {
    //	printf("currentPosition = setPosition\n");
    	return true;
    }
    
    elevation = m_pPotentiometerControl->FB(currentPosition, m_setPosition);
    //printf("current position %d " , currentPosition);
    //printf("setposition %d " , m_setPosition);
    //printf("elevation %f \n" , elevation);
    
    if (elevation < 0 && currentPosition <= m_lowerLimit)
    {
    	elevation = 0;
    	//printf("lowerlimit && elevation < 0\n");
    }
    if (elevation > 0 && currentPosition >= m_upperLimit) 
    {
    	elevation = 0;
    	//printf("upperlimit && elevation > 0\n");
    }
    if(elevation < 1e-3 && elevation > -1e-3)
    {
    	m_pElevator->Set(0);
        return true;
    }
    m_pElevator->Set(elevation);
    return false;
}

void PotentiometerControl::ManualControl(float stickInput, float sensitivity)
{
    // stickInput from [-1 to 1]
    float onePercent = (float)(m_upperLimit - m_lowerLimit)/(100.f);
    
    m_setPosition = (uint16_t) (m_pElevation->GetValue() + (stickInput * onePercent * sensitivity));
    
    assureBounds();
}

void PotentiometerControl::SetPercent(float setPoint)
{
    m_setPosition = (uint16_t) (m_lowerLimit + (setPoint)*(m_upperLimit - m_lowerLimit));
    assureBounds();
}

void PotentiometerControl::SetPosition(uint16_t setPoint)
{
    m_setPosition = setPoint;
    assureBounds();
}

/*void PotentiometerControl::SetAngle(float angle)
{
    
}*/

float PotentiometerControl::GetPercent()
{
    return m_pElevation->GetValue() / ((float)m_upperLimit - m_lowerLimit);
}

uint16_t PotentiometerControl::GetPosition()
{
    return m_pElevation->GetValue();
}

/*float PotentiometerControl::GetAngle()
{
 
}*/

void PotentiometerControl::assureBounds()
{
    if(m_setPosition > m_upperLimit)
        m_setPosition = m_upperLimit;
    if(m_setPosition < m_lowerLimit)
        m_setPosition = m_lowerLimit;
}




