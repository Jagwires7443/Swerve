// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/CommandPtr.h>

#include "commands/AutonomousCommands.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/TransferArmSubsystem.h"
#include "subsystems/Infrastructure.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/AmpSubsystem.h"
#include <frc/shuffleboard/Shuffleboard.h>

#include <memory>
#include <tuple>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer
{
public:
  RobotContainer() noexcept;

  RobotContainer(const RobotContainer &) = delete;
  RobotContainer &operator=(const RobotContainer &) = delete;

#pragma region Autonomous
public:
  void AutonomousInit() noexcept;
  void AutonomousPeriodic() noexcept;
  void AutonomousExit() noexcept;
  std::optional<frc2::CommandPtr> GetAutonomousCommand() noexcept;
#pragma endregion

#pragma region Teleop
public:
  void TeleopInit() noexcept;
  void TeleopPeriodic() noexcept;
  void TeleopExit() noexcept;

private:
  static frc2::CommandPtr DriveCommandFactory(RobotContainer *container) noexcept;
  std::tuple<double, double, double, bool> GetDriveTeleopControls() noexcept;
  double ConditionRawTriggerInput(double RawTrigVal) noexcept;
  double ConditionRawJoystickInput(double RawJoystickVal, double mixer = 0.75) noexcept;
  void ConfigureBindings() noexcept;

  bool m_fieldOriented{true};
  bool m_lock{false};
  bool triggerSpeedEnabled{false};
  
  frc2::CommandXboxController m_xboxDrive{0};
  frc2::CommandXboxController m_xboxOperate{1};
  
  //Code for adding the controllers
  frc::POVButton dpadUp{&m_xboxOperate, 0}; //0 degree for pulling in note to Amp
  frc::POVButton dpadRight{&m_xboxOperate, 90} //90 degree to right for extending Amp 
  frc::POVButton dpadDown{&m_xboxOperate, 180} //180 degree for releasing the note from the Amp
  frc::POVButton dpadLeft{&m_xboxOperate, 270}  //270 degrees for retracting the Amp

#pragma endregion

#pragma region Test
public:
  void TestInit() noexcept;
  void TestPeriodic() noexcept;
  void TestExit() noexcept;
private:
  static frc2::CommandPtr PointCommandFactory(RobotContainer *container) noexcept;
#pragma endregion

#pragma region Disabled
public:
  void DisabledInit() noexcept;
  void DisabledPeriodic() noexcept;
  void DisabledExit() noexcept;
#pragma endregion

#pragma region Subsystem Declare
private:
  // The robot's subsystems and commands are declared here
  DriveSubsystem m_driveSubsystem;
  InfrastructureSubsystem m_infrastructureSubsystem;
  IntakeSubsystem m_intakeSubsystem;
  TransferArmSubsystem m_transferArmSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  ClimberSubsystem m_climberSubsystem;
  AmpSubsystem m_ampSubsystem;

  // declared for the infrastructure subsystem
  uint m_LEDPattern{29};
  uint m_LEDPatternCount{0};
#pragma endregion
};
