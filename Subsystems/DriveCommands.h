//
//  DriveCommands.h
//  First 2013
//
//  Created by Kyle Rokos on 2/7/13.
//  Copyright (c) 2013 Kyle Rokos. All rights reserved.
//

#ifndef __First_2013__DriveCommands__
#define __First_2013__DriveCommands__

#include <iostream>
#include "../Utilities/Motion.h"
#include "../Utilities/PosCntl.h"

class DriveTrain;

namespace Drive
{
    // class DriveCommand - all types derive from this
    class DriveCommand
    {
    public:
        DriveCommand(DriveTrain* drive);
        virtual ~DriveCommand() {};
        
        bool Update();
        virtual bool CommandInit() = 0;
        virtual bool CommandUpdate() = 0;
        
    protected:
        DriveTrain *m_driveTrain;
        bool m_firstUpdate;
    };
    
    
    // class Distance - drive for a specified distance or distance and time
    class Distance : public DriveCommand
    {
    public:
        Distance(DriveTrain *drive, float meters, float seconds);
        Distance(DriveTrain *drive, float meters);
        
        ~Distance();
        bool CommandInit();
        bool CommandUpdate();
        
    protected:
        float m_meters;
        float m_seconds;
        Motion *m_pMotionDriveLeft;
        Motion *m_pMotionDriveRight;
    };

    
    // class Rotate - rotate in place for degrees
    class Rotate : public DriveCommand
    {
    public:
        Rotate(DriveTrain *drive, float degrees);
        bool CommandInit();
        bool CommandUpdate();
    protected:
        float m_degrees;
        Motion *m_pMotion;
    };
};


#endif /* defined(__First_2013__DriveCommands__) */
