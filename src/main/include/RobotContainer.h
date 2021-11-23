// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>

#include "commands/AutonomousCommands.h"
#include "commands/TestModeCommands.h"
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

  RobotContainer(const RobotContainer &) = delete;
  RobotContainer &operator=(const RobotContainer &) = delete;

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

  std::unique_ptr<frc2::RunCommand> m_driveCommand;
  std::unique_ptr<frc2::RunCommand> m_pointCommand;

  std::unique_ptr<ExampleCommand> m_autonomousCommand;

  std::unique_ptr<ZeroCommand> m_zeroCommand;
  std::unique_ptr<MaxVAndATurningCommand> m_maxVAndATurningCommand;
  std::unique_ptr<MaxVAndADriveCommand> m_maxVAndADriveCommand;
  std::unique_ptr<XsAndOsCommand> m_xsAndOsCommand;
  std::unique_ptr<SquareCommand> m_squareCommand;
  std::unique_ptr<SpirographCommand> m_spirographCommand;
  std::unique_ptr<OrbitCommand> m_orbitCommand;
  std::unique_ptr<PirouetteCommand> m_pirouetteCommand;

  frc::XboxController m_xbox{0};
};
