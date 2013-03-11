//
//  DriveCommands.cpp
//  First 2013
//
//  Created by Kyle Rokos on 2/7/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#include "DriveCommands.h"
#include "DriveTrain.h"
//#include "math.h"

namespace Drive {
    
    DriveCommand::DriveCommand(DriveTrain *drive)
    :m_driveTrain(drive),
    m_firstUpdate(true)
    {
        
    }
    
    bool DriveCommand::Update()
    {
        if(m_firstUpdate)
        {
            m_firstUpdate=false;
            CommandInit();
            return CommandUpdate();
        }
        return CommandUpdate();
    }
    
    Distance::Distance(DriveTrain *drive, float meters, float seconds)
    :DriveCommand(drive),
    m_meters(meters),
    m_seconds(seconds),
    m_pMotionDriveLeft(new Motion(0.06)),
    m_pMotionDriveRight(new Motion(0.06))
    {
        
    }
    
    Distance::Distance(DriveTrain *drive, float meters)
    :DriveCommand(drive),
    m_meters(meters),
    m_seconds((meters / drive->kMaxVelocityMetersPerSecond) + 1),
    m_pMotionDriveLeft(new Motion(0.06)),
    m_pMotionDriveRight(new Motion(0.06))
    {
        
    }
    
    Distance::~Distance()
    {
        delete m_pMotionDriveLeft;
        delete m_pMotionDriveRight;
    }
    
    bool Distance::CommandInit()
    {
        m_driveTrain->StartEncoders();
        double currentTime = Timer::GetPPCTimestamp();
        
        m_pMotionDriveLeft->Reset(m_driveTrain->GetLeftEncoder(), currentTime, m_meters * m_driveTrain->kEncoderCountsPerMeter, m_seconds);
        m_pMotionDriveRight->Reset(m_driveTrain->GetRightEncoder(), currentTime,m_meters * m_driveTrain->kEncoderCountsPerMeter, m_seconds);
        return true;
    }
    
    bool Distance::CommandUpdate()
    {
        bool finished = false;
        
        double currentTime = Timer::GetPPCTimestamp();
        float leftPower = m_pMotionDriveLeft->AdjustVelocity(m_driveTrain->GetLeftEncoder(), currentTime);
        float rightPower = m_pMotionDriveRight->AdjustVelocity(m_driveTrain->GetRightEncoder(), currentTime);

        //if(leftPower < 0.01 && leftPower > -0.01 &&
        //   rightPower < 0.01 && rightPower > -0.01)
        if(leftPower == 0 && rightPower == 0)
        {
            leftPower = 0;
            rightPower = 0;
            finished = true;
        }
        printf("In CommandUpdate::Distance setDistance = %f powerL=%f, powerR=%f\n", m_meters, leftPower, rightPower);
        m_driveTrain->TankDrive(leftPower, rightPower);
        
        return finished;
    }
    
    Rotate::Rotate(DriveTrain *drive, float degrees)
    :Drive::DriveCommand(drive),
    m_degrees(degrees),
    m_pMotion(new Motion(0.06))
    {
        while(m_degrees > 180)
            m_degrees = m_degrees - 360;
        while(m_degrees < -180)
            m_degrees = m_degrees + 360;
    }
    
    bool Rotate::CommandInit()
    {
        double currentTime = Timer::GetPPCTimestamp();
        
        m_pMotion->Reset(0, currentTime, m_degrees, m_degrees / m_driveTrain->kMaxRotationDegreesPerSecond);
        m_driveTrain->ResetAngle();
        
        return true;
    }
    
    bool Rotate::CommandUpdate()
    {
        bool finished = false;
        
        double currentTime = Timer::GetPPCTimestamp();
        float angle = m_driveTrain->GetAngle();
        float power = m_pMotion->AdjustVelocity((int) angle, currentTime);

        printf("In CommandUpdate::Rotate setAngle = %f angle=%f, power=%f\n", m_degrees, angle, power);
        
        if(m_degrees - angle < 3 && m_degrees - angle > -3)
        {
            power = 0;
            finished = true;
        }
        m_driveTrain->TankDrive(-power, power);
        
        return finished;
    }
}
