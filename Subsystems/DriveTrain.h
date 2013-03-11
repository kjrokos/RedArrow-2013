//
//  DriveTrain.h
//  First 2013
//
//  Created by Kyle Rokos on 1/27/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#ifndef __First_2013__DriveTrain__
#define __First_2013__DriveTrain__

#include <iostream>
#include <queue>
#include "WPILib.h"
#include "DriveCommands.h"

class DriveTrain : public RobotDrive
{
public:
    static const float kEncoderCountsPerMeter;
    static const float kMaxVelocityMetersPerSecond;
    static const float kMaxRotationDegreesPerSecond;
    
public:
    DriveTrain(uint32_t leftMotorChannel, uint32_t rightMotorChannel, uint32_t leftEncoderA, uint32_t leftEncoderB, uint32_t rightEncoderA, uint32_t rightEncoderB, uint32_t gyroChannel);
    
    ~DriveTrain();
    
    void Reset();
    
    void StartEncoders();
    void StopEncoders();
    int GetLeftEncoder();
    int GetRightEncoder();
    
    void ResetAngle();
    float GetAngle();
    
    void DriveDistance(float meters);
    void DriveDistance(float meters, float seconds);
    
    void Rotate(float degrees);
    
    void ManualControl(float stickX, float stickY, bool arcade=true);
    
    bool Update();
    
private:

    bool m_manualControl;
    
    Encoder *m_pLeftDriveEncoder;
	Encoder *m_pRightDriveEncoder;
    Gyro *m_pGyro;

    std::queue<Drive::DriveCommand*> m_commandQueue;

};


#endif /* defined(__First_2013__DriveTrain__) */
