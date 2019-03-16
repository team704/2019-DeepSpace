/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
#define DISABLE_USBCAMERA

#include "Robot.h"

#include <iostream>
#include <frc/DriverStation.h>
#include <frc/smartdashboard/SmartDashboard.h>
#if !defined(DISABLE_USBCAMERA)
#include <cameraserver/CameraServer.h>
#endif

void Robot::RobotInit() {
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  m_ltFrontMotor.ConfigFactoryDefault();
  m_ltFollowerMotor.ConfigFactoryDefault();
  m_rtFrontMotor.ConfigFactoryDefault();
  m_rtFollowerMotor.ConfigFactoryDefault();
  m_ltFollowerMotor.Follow(m_ltFrontMotor);
  m_rtFollowerMotor.Follow(m_rtFrontMotor);
  m_ltFrontMotor.SetInverted(false);
  m_ltFollowerMotor.SetInverted(false);
  m_rtFrontMotor.SetInverted(false);
  m_rtFollowerMotor.SetInverted(false);
  m_ltFrontMotor.SetSensorPhase(true);
  m_rtFrontMotor.SetSensorPhase(true);
  m_ltIntakeMotor.SetInverted(true);
  m_rtIntakeMotor.SetInverted(true);
  m_diffDrive.SetRightSideInverted(true);
  m_ltFrontMotor.ConfigOpenloopRamp(kDriveRampTime);
  m_ltFollowerMotor.ConfigOpenloopRamp(kDriveRampTime);
  m_rtFrontMotor.ConfigOpenloopRamp(kDriveRampTime);
  m_rtFollowerMotor.ConfigOpenloopRamp(kDriveRampTime);
  m_standSolenoid.Set(frc::DoubleSolenoid::Value::kOff);
  
  #if !defined(DISABLE_USBCAMERA)
  frc::CameraServer::GetInstance()->StartAutomaticCapture();
  #endif

  #if 0
  m_rangeFinder.init();
  m_rangeFinder.setTimeout(500);
  #endif

  // Set lift motion range
  /*
  m_pidLiftController.SetInputRange(19 * kValueToInches,
                                    96 * kValueToInches);
  */
  // Begin PID control
  //m_pidLiftController.Disable();
  
  switch(frc::DriverStation::GetInstance().GetAlliance())
  {
    case frc::DriverStation::Alliance::kBlue:
      std::cout << "Alliance: Blue" << std::endl;
      break;
    
    case frc::DriverStation::Alliance::kRed:
      std::cout << "Alliance: Red" << std::endl;
      break;

    default:
      std::cerr << "Alliance: Invalid" << std::endl;
      break;
  }
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  static frc::DoubleSolenoid::Value last_state = frc::DoubleSolenoid::Value::kOff;
  static double last_voltage = 0.0;
  static unsigned int last_distance = 0.0;
  
  if(last_state != m_standSolenoid.Get())
  {
    switch(m_standSolenoid.Get())
    {
      case frc::DoubleSolenoid::Value::kForward:
        std::cout << "Stand Solenoid: Stand (FORWARD)" << std::endl;
        break;

      case frc::DoubleSolenoid::Value::kReverse:
        std::cout << "Stand Solenoid: Recline (REVERSE)" << std::endl;
        break;

      default:
        std::cout << "Stand Solenoid: Free (OFF)" << std::endl;
        break;
    }
  }
/*
  if(last_voltage + 0.1 <= m_ultrasonic.GetVoltage() ||
     last_voltage - 0.1 >= m_ultrasonic.GetVoltage())
  {
    last_voltage = m_ultrasonic.GetVoltage();
    std::cout << "Ultrasonic: " << last_voltage << " V, "
              << (last_voltage - 0.5) / 4.5 * (60.0 - 3.0) + 3.0 << " in"
              << std::endl;
  }
  */
#if 0
  unsigned int distance = m_rangeFinder.readRangeSingleMillimeters();
  if((last_distance + 10 <= distance || last_distance - 10 >= distance) &&
     distance < 2500)
  {
    std::cout << "Laser Range Finder: " << (double)distance / 25.4 <<  "in" << std::endl;
    last_distance = distance;
  }
  
  if(m_rangeFinder.timeoutOccurred())
  {
    std::cerr << "Laser Range Finder: TIMEOUT OCCURED" << std::endl;
  }
  #endif

  last_state = m_standSolenoid.Get();
}

void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString("Auto Selector",
  //     kAutoNameDefault);
  // std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  DetectController();

  // Automatically stand
  m_standSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }

  HandleDrive();
}

void Robot::TeleopInit() {
  DetectController();
  /*
  if(m_driverJoystick.GetType() == frc::GenericHID::HIDType::kHIDJoystick ||
     m_driverJoystick.GetType() == frc::GenericHID::HIDType::kXInputArcadePad ||
     m_driverJoystick.GetType() == frc::GenericHID::HIDType::kXInputArcadeStick ||
     m_driverXboxGamepad.GetType() == frc::GenericHID::HIDType::kHIDGamepad ||
     m_driverXboxGamepad.GetType() == frc::GenericHID::HIDType::kXInputGamepad)
  {
    if(m_driverJoystick.GetType() == frc::GenericHID::HIDType::kHIDJoystick)
    {
      if(m_driverJoystick.GetName().find("Dual Action") != std::string::npos)
      {
        std::cout << "Found: Playstation Controller" << std::endl;
        m_joystickType = JoystickType::kPlaystation;
      }
      else if(m_driverJoystick.GetName().find("Extreme 3D") != std::string::npos)
      {
        std::cout << "Found: Joystick" << std::endl;
        m_joystickType = JoystickType::kJoystick;
      }
      else
      {
        std::cout << "Found: Undefined HIDJoystick "
                  << "[" << m_driverJoystick.GetName() << "]" << std::endl;
        m_joystickType m_ultrasonic= JoystickType::kUnknown;
      }
    }
    else if(m_driverXboxGamepad.GetType() == frc::GenericHID::HIDType::kXInputGamepad)
    {
      std::cout << "Found: X-Box Controller" << std::endl;
      m_joystickType = JoystickType::kXbox;
    }
    else
    {
      m_joystickType = JoystickType::kUnknown;
      std::cout << "Found: Undefined Controller"
                << "(" << m_driverJoystick.GetType() << ") "
                << "[" << m_driverJoystick.GetName() << "]" << std::endl;
    }
    
  }
  else
  {
    std::cerr << "No driver controller found!" << std::endl;
  }
  */

  // Automatically stand
  m_standSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void Robot::TeleopPeriodic() {
  HandleDrive();
}

void Robot::TestPeriodic() {}

void Robot::DetectController()
{
  //frc::DriverStation::GetInstance().GetCou
  switch(m_driverJoystick.GetType())
  {
    case frc::GenericHID::HIDType::kHIDJoystick:
      if(m_driverJoystick.GetName().find("Dual Action") != std::string::npos)
      {
        m_joystickType = JoystickType::kPlaystation;
        std::cout << "Found: Playstation Controller" << std::endl;
      }
      else if(m_driverJoystick.GetName().find("Extreme 3D") != std::string::npos)
      {
        m_joystickType = JoystickType::kJoystick;
        std::cout << "Found: Joystick Controller" << std::endl;
      }
      else
      {
        m_joystickType = JoystickType::kUnknown;
        std::cout << "Found: Unknown HIDJoystick "
                  << "[" << m_driverJoystick.GetName() << "]"
                  << std::endl;
      }
      break;

    case frc::GenericHID::HIDType::kXInputGamepad:
      m_joystickType = JoystickType::kXbox;
      std::cout << "Found: Xbox Controller" << std::endl;
      break;
    
    default:
      m_joystickType = JoystickType::kUnknown;
      std::cerr << "Found: Unknown Controller"
                << "(" << m_driverJoystick.GetType() << ") "
                << "[" << m_driverJoystick.GetName() << "]"
                << std::endl;
  }

  /*
  if(m_driverJoystick.GetType() == frc::GenericHID::HIDType::kHIDJoystick ||
     m_driverJoystick.GetType() == frc::GenericHID::HIDType::kXInputArcadePad ||
     m_driverJoystick.GetType() == frc::GenericHID::HIDType::kXInputArcadeStick ||
     m_driverXboxGamepad.GetType() == frc::GenericHID::HIDType::kHIDGamepad ||
     m_driverXboxGamepad.GetType() == frc::GenericHID::HIDType::kXInputGamepad)
  {
    if(m_driverJoystick.GetType() == frc::GenericHID::HIDType::kHIDJoystick)
    {
      if(m_driverJoystick.GetName().find("Dual Action") != std::string::npos)
      {
        std::cout << "Found: Playstation Controller" << std::endl;
        m_joystickType = JoystickType::kPlaystation;
      }
      else if(m_driverJoystick.GetName().find("Extreme 3D") != std::string::npos)
      {
        std::cout << "Found: Joystick" << std::endl;
        m_joystickType = JoystickType::kJoystick;
      }
      else
      {
        std::cout << "Found: Undefined HIDJoystick "
                  << "[" << m_driverJoystick.GetName() << "]" << std::endl;
        m_joystickType = JoystickType::kUnknown;
      }
    }
    else if(m_driverXboxGamepad.GetType() == frc::GenericHID::HIDType::kXInputGamepad)
    {
      std::cout << "Found: X-Box Controller" << std::endl;
      m_joystickType = JoystickType::kXbox;
    }
    else
    {
      m_joystickType = JoystickType::kUnknown;
      std::cout << "Found: Undefined Controller"
                << "(" << m_driverJoystick.GetType() << ") "
                << "[" << m_driverJoystick.GetName() << "]" << std::endl;
    }
    
  }
  else
  {
    std::cerr << "No driver controller found!" << std::endl;
  }
  */
}

void Robot::HandleDrive()
{
  // Do processing for a joystick
  if(m_joystickType == JoystickType::kJoystick)
  {
    HandleJoystick();
  }
  // Do processing for an xbox gamepad
  else if(m_joystickType == JoystickType::kXbox)
  {
    HandleXbox();
  }
  // Do processing for a playstation gamepad
  else if(m_joystickType == JoystickType::kPlaystation)
  {
    HandlePlaystation();
  }
  else
  {
    HandleNone();
  }
}

void Robot::HandleJoystick()
{
  m_diffDrive.ArcadeDrive(m_driverJoystick.GetY() * -1,
                          m_driverJoystick.GetX());
    
  if(m_driverJoystick.GetPOV() == 0)
  {
    m_liftMotor.Set(ControlMode::PercentOutput, kLiftSpeed);
    //m_pidLiftController.Disable();
  }
  else if(m_driverJoystick.GetPOV() == 180)
  {
    m_liftMotor.Set(ControlMode::PercentOutput, kLowerSpeed);
    //m_pidLiftController.Disable();
  }
  /*
  else if(not m_pidLiftController.IsEnabled())
  {
    m_liftMotor.Set(ControlMode::PercentOutput, 0.0);
  }
  */
  else
  {
    m_liftMotor.Set(ControlMode::PercentOutput, 0.0);
  }

  if(m_driverJoystick.GetRawButton(kJoystickOutputButton) &&
     m_driverJoystick.GetRawButton(kJoystickIntakeButton))
  {
    std::cerr << "Joystick: Unable to intake and output at the same time!" << std::endl;
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, 0.0);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, 0.0);
  }
  // Output ball
  else if(m_driverJoystick.GetRawButton(kJoystickOutputButton))
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, kOutputSpeed);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, kOutputSpeed);
  }
  // Intake ball
  else if(m_driverJoystick.GetRawButton(kJoystickIntakeButton))
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, kIntakeSpeed);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, kIntakeSpeed);
  }
  else
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, 0.0);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, 0.0);
  }

  // Check if slider is in the up position
  if(m_driverJoystick.GetRawAxis(kJoystickStandAxis) <= 0.25)
  {
    m_standSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else if(m_driverJoystick.GetRawAxis(kJoystickStandAxis) >= 0.75)
  {
    m_standSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }

  if(m_driverJoystick.GetRawButton(kJoystickEjectButton))
  {
    //m_intakeSolenoid.Set(true);
    m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
  }
  else
  {
    //m_intakeSolenoid.Set(false);
    m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
}

void Robot::HandleXbox()
{
  m_diffDrive.ArcadeDrive(m_driverXboxGamepad.GetY(frc::GenericHID::JoystickHand::kRightHand) * -1,
                          m_driverXboxGamepad.GetX(frc::GenericHID::JoystickHand::kLeftHand));
  
  if(m_driverXboxGamepad.GetBumper(frc::GenericHID::JoystickHand::kLeftHand) &&
     m_driverXboxGamepad.GetBumper(frc::GenericHID::JoystickHand::kRightHand))
  {
    std::cerr << "Gamepad: Unable to raise and lower lift at the same time!" << std::endl;
  }
  else if(m_driverXboxGamepad.GetBumper(frc::GenericHID::JoystickHand::kLeftHand))
  {
    m_liftMotor.Set(ControlMode::PercentOutput, kLiftSpeed);
    //m_pidLiftController.Disable();
  }
  else if(m_driverXboxGamepad.GetBumper(frc::GenericHID::JoystickHand::kRightHand))
  {
    m_liftMotor.Set(ControlMode::PercentOutput, kLowerSpeed);
    //m_pidLiftController.Disable();
  }
  /*
  else if(not m_pidLiftController.IsEnabled())
  {
    m_liftMotor.Set(ControlMode::PercentOutput, 0.0);
  }
  */
  else
  {
    m_liftMotor.Set(ControlMode::PercentOutput, 0.0);
  }

  // Output ball
  if(m_driverXboxGamepad.GetPOV() == 0)
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, kOutputSpeed);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, kOutputSpeed);
  }
  // Intake ball
  else if(m_driverXboxGamepad.GetPOV() == 180)
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, kIntakeSpeed);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, kIntakeSpeed);
  }
  else
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, 0.0);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, 0.0);
  }

  // Check if left trigger is held down
  if(m_driverXboxGamepad.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand) > 0.25)
  {
    if(m_driverXboxGamepad.GetAButton())
    {
      //m_intakeSolenoid.Set(true);
      m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
      //m_intakeSolenoid.Set(false);
      m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }

    if(m_driverXboxGamepad.GetBButton())
    {
      m_liftPosition = LiftPosition::kLowerHatch;
      //m_pidLiftController.Enable();
    }

    if(m_driverXboxGamepad.GetStartButton() &&
       m_driverXboxGamepad.GetBackButton())
    {
      std::cerr << "Gamepad: Unable to stand and recline at the same time!" << std::endl;
    }
    else if(m_driverXboxGamepad.GetStartButton())
    {
      m_standSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if(m_driverXboxGamepad.GetBackButton())
    {
      m_standSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
  }
  else
  {
    //m_intakeSolenoid.Set(false);
    m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
}

void Robot::HandlePlaystation()
{
  m_diffDrive.ArcadeDrive(m_driverJoystick.GetRawAxis(kPlaystationVelocityAxis) * -1,
                          m_driverJoystick.GetRawAxis(kPlaystationRotationAxis));
  
  if(m_driverJoystick.GetRawButton(kPlaystationLiftButton) &&
     m_driverJoystick.GetRawButton(kPlaystationLowerButton))
  {
    std::cerr << "Gamepad: Unable to raise and lower lift at the same time!" << std::endl;
  }
  else if(m_driverJoystick.GetRawButton(kPlaystationLiftButton))
  {
    m_liftMotor.Set(ControlMode::PercentOutput, kLiftSpeed);
    //m_pidLiftController.Disable();
  }
  else if(m_driverJoystick.GetRawButton(kPlaystationLowerButton))
  {
    m_liftMotor.Set(ControlMode::PercentOutput, kLowerSpeed);
    //m_pidLiftController.Disable();
  }
  /*
  else if(not m_pidLiftController.IsEnabled())
  {
    m_liftMotor.Set(ControlMode::PercentOutput, 0.0);
  }
  */
  else
  {
    m_liftMotor.Set(ControlMode::PercentOutput, 0.0);
  }

  // Output ball
  if(m_driverJoystick.GetPOV() == 0)
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, kOutputSpeed);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, kOutputSpeed);
  }
  // Intake ball
  else if(m_driverJoystick.GetPOV() == 180)
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, kIntakeSpeed);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, kIntakeSpeed);
  }
  else
  {
    m_ltIntakeMotor.Set(ControlMode::PercentOutput, 0.0);
    m_rtIntakeMotor.Set(ControlMode::PercentOutput, 0.0);
  }

  // Check if left trigger is held down
  if(m_driverJoystick.GetRawButton(kPlaystationTriggerButton))
  {
    if(m_driverJoystick.GetRawButton(kPlaystationEjectButton))
    {
      //m_intakeSolenoid.Set(true);
      m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else
    {
      //m_intakeSolenoid.Set(false);
      m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }

    if(m_driverJoystick.GetRawButton(kPlaystationTestButton))
    {
      m_liftPosition = LiftPosition::kLowerHatch;
      //m_pidLiftController.Enable();
    }

    if(m_driverJoystick.GetRawButton(kPlaystationStandButton) &&
       m_driverJoystick.GetRawButton(kPlaystationReclineButton))
    {
      std::cerr << "Gamepad: Unable to stand and recline at the same time!" << std::endl;
    }
    else if(m_driverJoystick.GetRawButton(kPlaystationStandButton))
    {
      m_standSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
    }
    else if(m_driverJoystick.GetRawButton(kPlaystationReclineButton))
    {
      m_standSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
    }
  }
  else
  {
    //m_intakeSolenoid.Set(false);
    m_intakeSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
  }
}

void Robot::HandleNone()
{
  m_diffDrive.ArcadeDrive(0.0, 0.0);
}

Robot::LiftPIDOutput::LiftPIDOutput(TalonSRX &motor)
  : m_motor(motor)
{
}

void Robot::LiftPIDOutput::PIDWrite(double output)
{
  m_motor.Set(ControlMode::PercentOutput, output);
}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
