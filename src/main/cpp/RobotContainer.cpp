// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>

RobotContainer::RobotContainer() noexcept : m_autonomousCommand(&m_driveSubsystem)
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command

  // The robot's frame of reference is the standard unit circle, from
  // trigonometry.  However, the front of the robot is facing along the positve
  // X axis.  This means the poitive Y axis extends outward from the left (or
  // port) side of the robot.  Poitive rotation is counter-clockwise.  On the
  // other hand, as the controller is held, the Y axis is aligned with forward.
  // And, specifically, it is the negative Y axis which extends forward.  So,
  // the robot's X is the controolers inverted Y.  On the controller, the X
  // axis lines up with the robot's Y axis.  And, the controller's positive X
  // extends to the right.  So, the robot's Y is the controller's inverted X.
  // Finally, the other controller joystick is used for commanding rotation and
  // things work out so that this is also an inverted X axis.
  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand(
      [&]() -> void {
        m_driveSubsystem.Drive(
            -m_xbox.GetY(frc::GenericHID::JoystickHand::kLeftHand) * physical::kMaxDriveSpeed,
            -m_xbox.GetX(frc::GenericHID::JoystickHand::kLeftHand) * physical::kMaxDriveSpeed,
            -m_xbox.GetX(frc::GenericHID::JoystickHand::kRightHand) * physical::kMaxTurnRate,
            true);
      },
      {&m_driveSubsystem}));
}

void RobotContainer::ConfigureButtonBindings() noexcept
{
  frc2::JoystickButton(&m_xbox, static_cast<int>(frc::XboxController::Button::kBumperLeft))
      .WhenPressed(frc2::RunCommand([&]() -> void { m_feederSubsystem.Set(0.8); }, {&m_feederSubsystem}))
      .WhenReleased(frc2::RunCommand([&]() -> void { m_feederSubsystem.Set(0.0); }, {&m_feederSubsystem}));

  frc2::JoystickButton(&m_xbox, static_cast<int>(frc::XboxController::Button::kBumperRight))
      .WhenPressed(frc2::RunCommand([&]() -> void { m_shooterSubsystem.Set(1.0); }, {&m_shooterSubsystem}))
      .WhenReleased(frc2::RunCommand([&]() -> void { m_shooterSubsystem.Set(0.0); }, {&m_shooterSubsystem}));
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
      -m_xbox.GetY(frc::GenericHID::JoystickHand::kLeftHand) * physical::kMaxDriveSpeed,
      -m_xbox.GetX(frc::GenericHID::JoystickHand::kLeftHand) * physical::kMaxDriveSpeed,
      -m_xbox.GetX(frc::GenericHID::JoystickHand::kRightHand) * physical::kMaxTurnRate,
      true);
}
