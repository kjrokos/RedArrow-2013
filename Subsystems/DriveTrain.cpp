//
//  DriveTrain.cpp
//  First 2013
//
//  Created by Kyle Rokos on 1/27/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#include "DriveTrain.h"

const float DriveTrain::kEncoderCountsPerMeter = 1159;
const float DriveTrain::kMaxVelocityMetersPerSecond = 1.f;
const float DriveTrain::kMaxRotationDegreesPerSecond = 60;



DriveTrain::DriveTrain(uint32_t leftMotorChannel, uint32_t rightMotorChannel,
    uint32_t leftEncoderA, uint32_t leftEncoderB, uint32_t rightEncoderA, uint32_t rightEncoderB, uint32_t gyroChannel)
    :RobotDrive(leftMotorChannel, rightMotorChannel),
    m_manualControl(true),
    m_pLeftDriveEncoder(new Encoder(leftEncoderA, leftEncoderB, true)),
    m_pRightDriveEncoder(new Encoder(rightEncoderA, rightEncoderB, false)),
    m_pGyro(/*new Gyro(gyroChannel)*/)
{
	Reset();
}

DriveTrain::~DriveTrain()
{
    if(m_pLeftDriveEncoder)
        delete m_pLeftDriveEncoder;
    if(m_pRightDriveEncoder)
        delete m_pRightDriveEncoder;
    if(m_pGyro)
        delete m_pGyro;
    
    while(m_commandQueue.size() > 0)
    {
        delete m_commandQueue.front();
        m_commandQueue.pop();
    }
}

void DriveTrain::Reset()
{
	this->TankDrive(0.f,0.f);
	m_manualControl = true;
	SetSafetyEnabled(true);
    StopEncoders();

    while(m_commandQueue.size() > 0)
    {
        delete m_commandQueue.front();
        m_commandQueue.pop();
    }
}

void DriveTrain::StartEncoders()
{
    m_pLeftDriveEncoder->Start();
    m_pRightDriveEncoder->Start();
    m_pLeftDriveEncoder->Reset();
    m_pRightDriveEncoder->Reset();
}

void DriveTrain::StopEncoders()
{
    m_pLeftDriveEncoder->Stop();
    m_pRightDriveEncoder->Stop();
}

int DriveTrain::GetLeftEncoder()
{
    return m_pLeftDriveEncoder->Get();
}

int DriveTrain::GetRightEncoder()
{
    return m_pLeftDriveEncoder->Get();
}

void DriveTrain::ResetAngle()
{
    //m_pGyro->Reset();
}

float DriveTrain::GetAngle()
{
    return 0;//m_pGyro->GetAngle();
}

void DriveTrain::DriveDistance(float meters)
{
    m_manualControl = false;
    m_commandQueue.push(new Drive::Distance(this,meters));
}

void DriveTrain::DriveDistance(float meters, float seconds)
{
    m_manualControl = false;
    m_commandQueue.push(new Drive::Distance(this,meters,seconds));
}

void DriveTrain::Rotate(float degrees)
{
    m_manualControl = false;
    m_commandQueue.push(new Drive::Rotate(this, degrees));
}

void DriveTrain::ManualControl(float stickX, float stickY, bool arcade)
{
    if(arcade==true)
    {
        this->ArcadeDrive(-stickX, stickY, true);
    }
    else
    {
        this->TankDrive(-stickX, -stickY);
    }
    
    m_manualControl = true;
}

bool DriveTrain::Update()
{
    if(m_manualControl)
        return true;
    
    printf("Command Queue Size %d\n", m_commandQueue.size()); 
    
    if(m_commandQueue.size() == 0)
        return true;
    
    bool finished = (m_commandQueue.front())->Update();
    
    if(finished)
    {
        delete m_commandQueue.front();
        m_commandQueue.pop();
    }
    return m_commandQueue.size() == 0;
}
