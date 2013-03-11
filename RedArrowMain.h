#include "WPILib.h"

#include "Utilities/CxTimer.h"
#include "Utilities/AutonomousManager.h"
#include "Subsystems/ArmControl.h"
#include "Subsystems/FeederControl.h"
#include "Subsystems/TwoStateServoControl.h"
#include "Subsystems/PotentiometerControl.h"
#include "Subsystems/DriveTrain.h"

//#define ARM_TYPE_POT

class BuiltinDefaultCode : public IterativeRobot
{
public:
/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
	BuiltinDefaultCode(void);
	~BuiltinDefaultCode(void);
	
	/********************************** Init Routines *************************************/
	void RobotInit(void);
	void DisabledInit(void);
	void AutonomousInit(void);
	void TeleopInit(void);

	/********************************** Periodic Routines *************************************/
	void DisabledPeriodic(void);
	void AutonomousPeriodic(void);
	void TeleopPeriodic(void);
	
	void GetDS();
	void ResetSubsystems();
	bool UpdateSubsystems();
	
	
//private:
	// Declare variale for the robot drive system
	//RobotDrive *m_robotDrive;		// robot will use PWM 1-2 for drive motors
	DriveTrain *m_robotDrive;
	
	CxTimer *m_t1;
#ifdef ARM_TYPE_POT
	PotentiometerControl *m_arm;
#else
	ArmControl *m_arm;
#endif
	FeederControl *m_feeder;
	TwoStateServoControl *m_flag;
	TwoStateServoControl *m_unjammer;
	Talon *m_shooter;
	Talon *m_climber;
	PotentiometerControl *m_elevation;
	Gyro *m_gyro;
	
private:
	AutonomousManager<BuiltinDefaultCode> *m_autonomousManager;
	
	//Camera
	AxisCamera *camera;
		
	float RSy;
	float RSx;
	float RSz;
	bool RS_B1;
	bool RS_B2;
	bool RS_B3;
	bool RS_B4;
	bool RS_B5;
	bool RS_B6;
	bool RS_B7;
	bool RS_B8;
	bool RS_B9;
	bool RS_B10;
	bool RS_B11;
	
	float LSy;
	float LSx;
	float LSz;
	bool LS_B1;
	bool prev_LS_B2;
	bool LS_B2;
	bool LS_B3;
	bool LS_B4;
	bool LS_B5;
	bool LS_B6;
	bool LS_B7;
	bool LS_B8;
	bool LS_B9;
	bool LS_B10;
	bool LS_B11;
	
	float m_shooterSpeed;
	
	
	// Declare a variable to use to access the driver station object
	DriverStation *m_ds;						// driver station object
	SendableChooser *m_autonomousModeChooser;
	
	// Declare variables for the two joysticks being used
	Joystick *m_rightStick;			// joystick 1 (arcade stick or right tank stick)
	Joystick *m_leftStick;			// joystick 2 (tank left stick)
};

