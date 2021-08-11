// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "RobotContainer.h"

#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer() noexcept : m_autonomousCommand(&m_driveSubsystem)
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        m_driveSubsystem.Drive(
            m_xbox.GetX(frc::GenericHID::JoystickHand::kLeftHand) * physical::kMaxDriveSpeed,
            m_xbox.GetY(frc::GenericHID::JoystickHand::kLeftHand) * physical::kMaxDriveSpeed,
            m_xbox.GetX(frc::GenericHID::JoystickHand::kRightHand) * physical::kMaxTurnRate,
            false);
      },
      {&m_driveSubsystem}));
}

void RobotContainer::ConfigureButtonBindings() noexcept
{
  // Configure your button bindings here
}

frc2::Command *RobotContainer::GetAutonomousCommand() noexcept
{
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}

void RobotContainer::TestInit() noexcept
{
  m_driveSubsystem.TestInit();

  frc::SendableChooser<frc2::Command *> *chooser = m_driveSubsystem.TestModeChooser();
  chooser->SetDefaultOption("Zero", nullptr);
  chooser->AddOption("Xs and Os", nullptr);
  chooser->AddOption("Orbit", nullptr);
  chooser->AddOption("Spirograph", nullptr);
  chooser->AddOption("Drive", nullptr);
}

void RobotContainer::TestPeriodic() noexcept
{
  m_driveSubsystem.TestPeriodic();

  // XXX temp hack
  m_driveSubsystem.Drive(
      m_xbox.GetX(frc::GenericHID::JoystickHand::kLeftHand) * physical::kMaxDriveSpeed,
      m_xbox.GetY(frc::GenericHID::JoystickHand::kLeftHand) * physical::kMaxDriveSpeed,
      m_xbox.GetX(frc::GenericHID::JoystickHand::kRightHand) * physical::kMaxTurnRate,
      false);
}
