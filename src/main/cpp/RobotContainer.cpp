// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

#include <cmath>

RobotContainer::RobotContainer() noexcept : m_autonomousCommand(&m_driveSubsystem)
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command; non-owning pointer is passed by value.
  auto requirements = {dynamic_cast<frc2::Subsystem *>(&m_driveSubsystem)};

  m_driveCommand = std::make_unique<frc2::RunCommand>(
      [&]() -> void {
        const auto controls = GetDriveTeleopControls();

        m_driveSubsystem.Drive(
            std::get<0>(controls) * physical::kMaxDriveSpeed,
            std::get<1>(controls) * physical::kMaxDriveSpeed,
            std::get<2>(controls) * physical::kMaxTurnRate,
            true);
      },
      requirements);

  m_driveSubsystem.SetDefaultCommand(*m_driveCommand);
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

std::tuple<double, double, double> RobotContainer::GetDriveTeleopControls() noexcept
{
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
  double x = -m_xbox.GetY(frc::GenericHID::JoystickHand::kLeftHand);
  double y = -m_xbox.GetX(frc::GenericHID::JoystickHand::kLeftHand);
  double z = -m_xbox.GetX(frc::GenericHID::JoystickHand::kRightHand);

  // Add some deadzone, so the robot doesn't drive when the joysticks are
  // released and return to "zero".  These implement a continuous deadband, one
  // in which the full range of outputs may be generated, once joysticks move
  // outside the deadband.

  // Also, cube the result, to provide more opertor control.  Just cubing the
  // raw value does a pretty good job with the deadband, but doing both is easy
  // and guarantees no movement in the deadband.
  auto shape = [](double raw) -> double {
    constexpr double range = 0.05;
    constexpr double slope = 1.0 / (1.0 - range);

    if (raw > -range && raw < +range)
    {
      raw = 0.0;
    }
    else if (raw <= -range)
    {
      raw += range;
      raw *= slope;
    }
    else if (raw >= +range)
    {
      raw -= range;
      raw *= slope;
    }

    return std::pow(raw, 3.0);
  };

  return std::make_tuple(shape(x), shape(y), shape(z));
}

void RobotContainer::TestInit() noexcept
{
  m_driveSubsystem.TestInit();

  frc::SendableChooser<frc2::Command *> *chooser = m_driveSubsystem.TestModeChooser();

  chooser->SetDefaultOption("Zero", nullptr);
  chooser->AddOption("Xs and Os", nullptr);
  chooser->AddOption("Point", nullptr);
  chooser->AddOption("Orbit", nullptr);
  chooser->AddOption("Square", nullptr);
  chooser->AddOption("Spirograph", nullptr);
  chooser->AddOption("Drive", m_driveCommand.get());
}

void RobotContainer::TestExit() noexcept
{
  m_driveSubsystem.TestExit();
}

void RobotContainer::TestPeriodic() noexcept
{
  m_driveSubsystem.TestPeriodic();

  // The command scheduler does not ordinarily run in Test Mode, so run it here
  // in case any commands have been schedued.
  frc2::CommandScheduler::GetInstance().Run();
}
