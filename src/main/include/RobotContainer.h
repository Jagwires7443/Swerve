// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/Command.h>

#include "commands/ExampleCommand.h"
#include "subsystems/DriveSubsystem.h"

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
  void TestPeriodic() noexcept;

private:
  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_driveSubsystem;
  ExampleCommand m_autonomousCommand;
  frc::XboxController m_xbox{0};

  void ConfigureButtonBindings() noexcept;
};
