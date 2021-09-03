// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>

#include "commands/AutonomousCommands.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

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

  frc2::Command *GetAutonomousCommand() noexcept;

  void TestInit() noexcept;
  void TestExit() noexcept;
  void TestPeriodic() noexcept;

private:
  std::tuple<double, double, double, bool> GetDriveTeleopControls() noexcept;

  void ConfigureButtonBindings() noexcept;

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_driveSubsystem;
  FeederSubsystem m_feederSubsystem;
  ShooterSubsystem m_shooterSubsystem;

  ExampleCommand m_autonomousCommand;
  std::unique_ptr<frc2::RunCommand> m_driveCommand;

  frc::XboxController m_xbox{0};
};
