// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer() noexcept : m_autonomousCommand(&m_driveSubsystem)
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command
  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand(
      [this]() {
        m_driveSubsystem.Drive(units::meters_per_second_t{0},  // m_driverController.GetY(frc::GenericHID::kLeftHand)),
                               units::meters_per_second_t{0},  // m_driverController.GetY(frc::GenericHID::kRightHand)),
                               units::radians_per_second_t{0}, // m_driverController.GetX(frc::GenericHID::kLeftHand)),
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
}

void RobotContainer::TestPeriodic() noexcept
{
  m_driveSubsystem.TestPeriodic();
}
