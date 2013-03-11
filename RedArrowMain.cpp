#include "RedArrowMain.h"


#define LEFT_DRIVE_PWM 1
#define RIGHT_DRIVE_PWM 2
#define ARM_PWM 3
#define SHOOTER_PWM 4
#define CLIMBER_PWM 5 
#define ELEVATION_PWM 6

#define FLAG_SERVO 9
#define UNJAMMER_SERVO 10

#define FEEDER_RELAY 1

#define ARM_UP_LS 1
#define ARM_DOWN_LS 2
#define FEEDER_LS 3
#define LEFT_DRIVE_ENC_A 6
#define LEFT_DRIVE_ENC_B 7
#define RIGHT_DRIVE_ENC_A 8
#define RIGHT_DRIVE_ENC_B 9

#define GYRO 1
#define ELEVATION_POT 2
#define ARM_POT 3



NextState AutonomousProgramA(BuiltinDefaultCode *robot, int32_t state)
{
	switch(state)
	{
	case 0:
		//robot->m_elevation->SetPosition(465);
		robot->m_elevation->SetPosition(435);
		robot->m_shooter->Set(1);
		robot->m_feeder->ResetNumberOfFeeds();
		robot->m_unjammer->Lower();
		//robot->m_robotDrive->SetSafetyEnabled(false);
		//robot->m_robotDrive->DriveDistance(1);
		//robot->m_robotDrive->Rotate(90);
		
		return NextState(1,2,5);
		break;
	case 1:
		if(robot->m_feeder->GetNumberOfFeeds() >= 3)
		{
			robot->m_shooter->Set(0);
			robot->m_unjammer->Raise();
			return NextState::EndState();
		}
		robot->m_feeder->Feed();
		return NextState(1,1,1);
		break;
	default:
		break;
	}
	return NextState::EndState();
};

//FIX ARM CONTROL TO USE POT
NextState AutonomousProgramB(BuiltinDefaultCode *robot, int32_t state)
{
	switch(state)
	{
	case 0:
		robot->m_robotDrive->SetSafetyEnabled(false);
		robot->m_robotDrive->DriveDistance(-2);
		robot->m_feeder->ResetNumberOfFeeds();
		robot->m_elevation->SetPosition(510);
		return NextState(1,1);
		break;
	case 1:
		robot->m_shooter->Set(1);
#ifdef ARM_TYPE_POT
		robot->m_arm->SetPosition(200);
#else
		robot->m_arm->ArmDown();
#endif
		return NextState(2,2);
		break;
	case 2:
		if(robot->m_feeder->GetNumberOfFeeds() >= 3)
			{
			robot->m_shooter->Set(0);
			return NextState(3,2);
			}
			robot->m_feeder->Feed();
		return NextState(2,1);
		break;
	case 3:
		robot->m_elevation->SetPosition(450);
		robot->m_robotDrive->DriveDistance(2);
		return NextState(4,1);
		break;
	case 4:
#ifdef ARM_TYPE_POT
		robot->m_arm->SetPosition(100);
#else
		robot->m_arm->ArmUp();
#endif
		robot->m_feeder->ResetNumberOfFeeds();
		return NextState(5,1);
		break;
	case 5:
		robot->m_shooter->Set(1);
		return NextState(6,1);
		break;
	case 6:
		if(robot->m_feeder->GetNumberOfFeeds() >= 2)
		{
			robot->m_shooter->Set(0);
			return NextState::EndState();
		}
		robot->m_feeder->Feed();
		return NextState(6,2);
		break;
	default:
			break;
	}
	return NextState::EndState();
};


/**
 * Constructor for this "BuiltinDefaultCode" Class.
 * 
 * The constructor creates all of the objects used for the different inputs and outputs of
 * the robot.  Essentially, the constructor defines the input/output mapping for the robot,
 * providing named objects for each of the robot interfaces. 
 */
BuiltinDefaultCode::BuiltinDefaultCode(void)	
{

	// Create a robot using standard right/left robot drive on PWMS 1, 2,
	m_robotDrive = //new RobotDrive(LEFT_DRIVE_PWM, RIGHT_DRIVE_PWM);
				new DriveTrain(LEFT_DRIVE_PWM, RIGHT_DRIVE_PWM,LEFT_DRIVE_ENC_A, LEFT_DRIVE_ENC_B, RIGHT_DRIVE_ENC_A, RIGHT_DRIVE_ENC_B, GYRO);
	
	m_feeder = new FeederControl(FEEDER_RELAY, UNJAMMER_SERVO, FEEDER_LS);
#ifdef ARM_TYPE_POT
	m_arm = new PotentiometerControl(ARM_PWM, ARM_POT, 70, 600);
#else
	m_arm = new ArmControl(ARM_PWM, ARM_UP_LS, ARM_DOWN_LS);
#endif
	m_shooter = new Talon(SHOOTER_PWM);
	m_climber = new Talon(CLIMBER_PWM);
	m_elevation = new PotentiometerControl(ELEVATION_PWM, ELEVATION_POT, 270, 900, 0.06);
	m_unjammer = new TwoStateServoControl(UNJAMMER_SERVO, 0, .915);
	m_flag = new TwoStateServoControl(FLAG_SERVO, 0.66, 0.10);
	m_gyro = new Gyro(1);
	
	// Initialize AutonomousManager
	m_autonomousManager = new AutonomousManager<BuiltinDefaultCode>(this, 0, 0);
	
	// Acquire the Driver Station object
	m_ds = DriverStation::GetInstance();
	
	m_autonomousModeChooser = new SendableChooser();
	
	// Define joysticks being used at USB port #1 and USB port #2 on the Drivers Station
	m_rightStick = new Joystick(1);
	m_leftStick = new Joystick(2);
	m_t1=new CxTimer();
	m_t1->Reset();

	m_shooterSpeed=0;

	ResetSubsystems();

}

BuiltinDefaultCode::~BuiltinDefaultCode(void)
{
	delete m_robotDrive;
	delete m_arm;
	delete m_feeder;
	delete m_elevation;
	delete m_climber;
	delete m_ds;
	delete m_rightStick;
	delete m_leftStick;
	delete m_flag;
	delete m_unjammer;
	delete m_t1;
	delete m_shooter;
	delete m_autonomousManager;
	delete m_gyro;
	delete m_autonomousModeChooser;
}


/********************************** Init Routines *************************************/

void BuiltinDefaultCode::RobotInit(void) 
{
	// Actions which would be performed once (and only once) upon initialization of the
	// robot would be put here.
	
	m_autonomousModeChooser->AddDefault("A: Shoot from back side", new std::string("side"));
	m_autonomousModeChooser->AddObject("B: Shoot from middle", new std::string("middle"));
	SmartDashboard::PutData("Autonomous Mode", m_autonomousModeChooser);
	//Camera Initalization
#ifdef CAMERA_ON

	camera->GetInstance("10.32.34.11");
	camera->WriteResolution(AxisCamera::kResolution_320x240);
	camera->WriteCompression(20);
	camera->WriteBrightness(0);
	camera->WriteColorLevel(50);
	Wait(3.0);
#endif

}

void BuiltinDefaultCode::DisabledInit(void) 
{
}

void BuiltinDefaultCode::AutonomousInit(void) 
{
	ResetSubsystems();
	std::string mode = *((std::string*)m_autonomousModeChooser->GetSelected());
	SmartDashboard::PutString("Autonomous Mode", mode);
	if(mode == "middle")
		m_autonomousManager->SetStartState(AutonomousProgramB, 0);
	else
		m_autonomousManager->SetStartState(AutonomousProgramA, 0);
	
	//m_autonomousManager->SetStartState(AutonomousProgramA, 0);
}

void BuiltinDefaultCode::TeleopInit(void) 
{	
	ResetSubsystems();
}

/********************************** Periodic Routines *************************************/

void BuiltinDefaultCode::DisabledPeriodic(void)  
{	
}

void BuiltinDefaultCode::AutonomousPeriodic(void) 
{
	CxTimer::Update();
	
	m_autonomousManager->Run();
	
}


void BuiltinDefaultCode::TeleopPeriodic(void) 
{
	CxTimer::Update();

	GetDS();
	bool leftJoystickIsUsed = false;

	
	if(LS_B5) // if left stick is micro-adjusting, prioritize over right stick
	{
		float moveValue=LSy/2;
		float rotateValue=LSx/2;
		m_robotDrive->ArcadeDrive(-moveValue,-rotateValue);
		leftJoystickIsUsed = true;
	}
	else if(RS_B5 && !LS_B5) // right stick is micro-adjusting (and left stick is not)
	{
		float moveValue=RSy/2;
		float rotateValue=RSx/2;
		m_robotDrive->ArcadeDrive(-moveValue,-rotateValue);
	}
	else // right stick normal drive mode
	{
		m_robotDrive->ArcadeDrive(-RSy ,-RSx);			// drive with arcade style (use right stick)
	}

	// do logic for decisions
	if(LS_B1)			//Shoot Frisbee
		m_feeder->Feed();
	if(LS_B6)					//Unjam Frisbee
		m_feeder->Reverse();

	if(LS_B2 && leftJoystickIsUsed == false)		//Elevation Control
	{
		m_elevation->ManualControl(LSy, 30);
		leftJoystickIsUsed = true;
	}
	else if(prev_LS_B2 && !LS_B2)
	{
		m_elevation->Reset();
	}

	if(LS_B7)					//Restack Frisbees
		m_unjammer->Lower();
	//if(!LS_B7)					//UnReStack Frisbees
		//m_unjammer->Lower();
	
	/*
	if(LS_B7)
	{
		m_feeder->UnjammerDown();
	}
	*/
	if(LS_B9)		//Elevation Preset Rear
		{
		m_elevation->SetPosition(435);
		//m_elevation->SetPosition(465);
		//m_shooterSpeed = 1;
		}
		

	if(LS_B8)		//Elevation Preset Front
		{
		m_elevation->SetPosition(495);
		//m_shooterSpeed = 1;
		}
		
	if(RS_B11)		//Elevation Preset Loading
	{
		m_elevation->SetPosition(780);
	}
//	if(RS_B6)	//Arm Down
//	{
//#ifdef ARM_TYPE_POT
//		m_arm->SetPosition(100);
//#else
//		m_arm->ArmDown();
//#endif
//	}
//	if(RS_B7)		//Arm Up
//		{
//#ifdef ARM_TYPE_POT
//		m_arm->SetPosition(350);
//#else
//		m_arm->ArmUp();
//#endif
//		}
	

	if(LS_B3 && leftJoystickIsUsed == false)		//Arm Manual Control
	{
		m_arm->ManualControl(LSy);
		leftJoystickIsUsed = true;
	}

	if(RS_B8)		//Flag Lower
		{
		m_flag->Lower();
		}

	if(RS_B9)		//Flag Raise
		{
		m_flag->Raise();
		}

	if(LS_B11)		//Frisbee On
		{
		m_shooterSpeed = (-LSz+1)/2.f;
		m_unjammer->Lower();
		}

	if(LS_B10)		//Frisbee Off
		{
		m_shooterSpeed = 0;
		m_unjammer->Raise();
		}

	if(m_shooterSpeed)
		printf("m_shooterSpeed: %f", m_shooterSpeed);

	m_shooter->Set(m_shooterSpeed);

	if(LS_B4 && leftJoystickIsUsed == false)		//Climber Manual Control
	{
		m_climber->Set(LSy);
		leftJoystickIsUsed = true;
	}
	else 
	{
		m_climber->Set(0);
	}
	
	SmartDashboard::PutNumber("Frisbee", m_shooterSpeed);
	SmartDashboard::PutNumber("PotentiometerElevation", (uint16_t) m_elevation->GetPosition());

#ifdef ARM_TYPE_POT
	SmartDashboard::PutNumber("PotentiometerArm", (uint16_t) m_arm->GetPosition());
#endif

	UpdateSubsystems();
} // TeleopPeriodic(void)

void BuiltinDefaultCode::GetDS()
{
	RSx=m_rightStick->GetX();
	RSy=-m_rightStick->GetY();
	RSz=m_rightStick->GetZ();
	RS_B1=m_rightStick->GetRawButton(1);
	RS_B2=m_rightStick->GetRawButton(2);
	RS_B3=m_rightStick->GetRawButton(3);
	RS_B4=m_rightStick->GetRawButton(4);
	RS_B5=m_rightStick->GetRawButton(5);
	RS_B6=m_rightStick->GetRawButton(6);
	RS_B7=m_rightStick->GetRawButton(7);
	RS_B8=m_rightStick->GetRawButton(8);
	RS_B9=m_rightStick->GetRawButton(9);
	RS_B10=m_rightStick->GetRawButton(10);
	RS_B11=m_rightStick->GetRawButton(11);

	LSx=m_leftStick->GetX();
	LSy=-m_leftStick->GetY();
	LSz=m_leftStick->GetZ();
	LS_B1=m_leftStick->GetRawButton(1);
	prev_LS_B2 = LS_B2;
	LS_B2=m_leftStick->GetRawButton(2);
	LS_B3=m_leftStick->GetRawButton(3);
	LS_B4=m_leftStick->GetRawButton(4);
	LS_B5=m_leftStick->GetRawButton(5);
	LS_B6=m_leftStick->GetRawButton(6);
	LS_B7=m_leftStick->GetRawButton(7);
	LS_B8=m_leftStick->GetRawButton(8);
	LS_B9=m_leftStick->GetRawButton(9);
	LS_B10=m_leftStick->GetRawButton(10);
	LS_B11=m_leftStick->GetRawButton(11);


	SmartDashboard::PutNumber("RSx",(double)RSx);
	SmartDashboard::PutNumber("RSy",(double)RSy);
	SmartDashboard::PutNumber("RSz", (double)RSz);

	SmartDashboard::PutNumber("LSx",(double)LSx);
	SmartDashboard::PutNumber("LSy",(double)LSy);
	SmartDashboard::PutNumber("LSz", (double)LSz);
	
	SmartDashboard::PutNumber("CodeVersion", 5);

}

void BuiltinDefaultCode::ResetSubsystems()
{
	RSx=0;
	RSy=0;
	RSx=0;
	RS_B1=false;
	RS_B2=false;
	RS_B3=false;
	RS_B4=false;
	RS_B5=false;
	RS_B6=false;
	RS_B7=false;
	RS_B8=false;
	RS_B9=false;
	RS_B10=false;
	RS_B11=false;
	LSx=0;
	LSy=0;
	LSx=0;
	LS_B1=false;
	LS_B2=false;
	LS_B3=false;
	LS_B4=false;
	LS_B5=false;
	LS_B6=false;
	LS_B7=false;
	LS_B8=false;
	LS_B9=false;
	LS_B10=false;
	LS_B11=false;
	
	m_arm->Reset();
	m_feeder->Reset();
	m_flag->Reset();
	m_elevation->Reset();
	m_robotDrive->Reset();
	m_shooterSpeed = 0;
	m_shooter->Set(0);
	m_climber->Set(0);
	m_unjammer->Reset();
	m_unjammer->Raise();
}

bool BuiltinDefaultCode::UpdateSubsystems()
{
	bool finished = true;
	finished = m_arm->Update()    && finished;
	finished = m_feeder->Update() && finished;
	finished = m_flag->Update()   && finished;
	finished = m_unjammer->Update() &&finished;
	finished = m_elevation->Update() && finished;
	finished = m_robotDrive->Update() && finished;
	return finished;
}

START_ROBOT_CLASS(BuiltinDefaultCode);
