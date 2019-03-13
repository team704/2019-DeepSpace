/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/AnalogInput.h>
#include <frc/Solenoid.h>
#include <frc/DoubleSolenoid.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/PIDController.h>
#include <ctre/Phoenix.h>
#include "VL53L0X.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;

 private:
  enum LiftMode
  {
    kManual = 0,
    kClosedLoop = 1
  };

  enum LiftPosition
  {
    kInvalid = -1,
    kLowerHatch = 0,
    kMiddleHatch = 1,
    kUpperHatch = 2,
    kHabCargo = 3,
    kLowerCargo = 4,
    kMiddleCargo = 5,
    kUpperCargo = 6,
    kLoadingCargo = 7
  };

  enum JoystickType
  {
    kNone = -1,
    kUnknown = 0,
    kXbox = 1,
    kJoystick = 2,
    kPlaystation = 3
  };

  class LiftPIDOutput : public frc::PIDOutput
  {
   public:
    explicit LiftPIDOutput(TalonSRX &motor);

    void PIDWrite(double output) override;

   private:
    TalonSRX &m_motor;
  };

  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  LiftMode m_liftMode{LiftMode::kManual};
  LiftPosition m_liftPosition{LiftPosition::kInvalid};
  JoystickType m_joystickType{JoystickType::kNone};

  // Distance in inches the robot wants to stay from an object
  static constexpr int kHoldDistance = 12;

  // Factor to convert sensor values to a distance in inches
  static constexpr double kValueToInches = 0.125;
  static constexpr double kLiftSpeed = 1.0;
  static constexpr double kLowerSpeed = -1.0;
  static constexpr double kIntakeSpeed = 1.0;
  static constexpr double kOutputSpeed = -0.3;

  static constexpr int kJoystickVelocityAxis = 1;
  static constexpr int kJoystickRotationAxis = 2;
  
  static constexpr int kGamepadVelocityAxis = 1;
  static constexpr int kGamepadRotationAxis = 3;

  static constexpr int kPlaystationVelocityAxis = 3;
  static constexpr int kPlaystationRotationAxis = 0;

  static constexpr int kPlaystationLiftButton = 5;
  static constexpr int kPlaystationLowerButton = 6;
  static constexpr int kPlaystationTriggerButton = 7;
  static constexpr int kPlaystationEjectButton = 2;
  static constexpr int kPlaystationStandButton = 10;
  static constexpr int kPlaystationReclineButton = 9;
  static constexpr int kPlaystationTestButton = 3;

  static constexpr int kJoystickIntakeButton = 2;
  static constexpr int kJoystickOutputButton = 1;
  static constexpr int kJoystickEjectButton = 3;
  static constexpr int kJoystickStandAxis = 3;

  static constexpr int kLtFrontMotorPort = 2;
  static constexpr int kLtFollowerMotorPort = 1;
  static constexpr int kRtFrontMotorPort = 7;
  static constexpr int kRtFollowerMotorPort = 6;

  static constexpr int kLtIntakeMotorPort = 4;
  static constexpr int kRtIntakeMotorPort = 5;

  static constexpr int kLiftMotorPort = 3;

  static constexpr int kIntakeExtendSolenoidPort = 2;
  static constexpr int kIntakeRetractSolenoidPort = 3;
  static constexpr int kStandSolenoidPort = 1;
  static constexpr int kReclineSolenoidPort = 0;

  static constexpr int kUltrasonicPort = 3;

  static constexpr int kJoystickPort = 0;

  // proportional speed constant
  static constexpr double kP = 7.0;

  // integral speed constant
  static constexpr double kI = 0.018;

  // derivative speed constant
  static constexpr double kD = 1.5;

  WPI_TalonSRX m_ltFrontMotor{kLtFrontMotorPort};
  WPI_TalonSRX m_ltFollowerMotor{kLtFollowerMotorPort};
  WPI_TalonSRX m_rtFrontMotor{kRtFrontMotorPort};
  WPI_TalonSRX m_rtFollowerMotor{kRtFollowerMotorPort};

  TalonSRX m_ltIntakeMotor{kLtIntakeMotorPort};
  TalonSRX m_rtIntakeMotor{kRtIntakeMotorPort};

  TalonSRX m_liftMotor{kLiftMotorPort};

  VL53L0X m_rangeFinder{0x29};

  //LiftPIDOutput m_pidLiftOutput{m_liftMotor};

  //frc::PIDController m_pidLiftController{kP, kI, kD, m_ultrasonic, m_pidLiftOutput};

  frc::DoubleSolenoid m_intakeSolenoid{kIntakeRetractSolenoidPort,
                                       kIntakeExtendSolenoidPort};

  frc::DoubleSolenoid m_standSolenoid{kStandSolenoidPort,
                                      kReclineSolenoidPort};

  frc::AnalogInput m_ultrasonic{kUltrasonicPort};

  frc::DifferentialDrive m_diffDrive{m_ltFrontMotor, m_rtFrontMotor};

  frc::XboxController m_driverXboxGamepad{0};

  frc::Joystick m_driverJoystick{0};

  frc::XboxController m_secondaryGamepad{1};

  frc::Joystick m_secondaryJoystick{1};

  void DetectController();

  void HandleDrive();

  void HandleJoystick();

  void HandleXbox();

  void HandlePlaystation();

  void HandleNone();
  
  //MyPIDOutput m_pidOutput{m_liftMotor};

  //frc::PIDController m_pidController{kP, kI, kD, m_ultrasonic, m_pidOutput};
};
