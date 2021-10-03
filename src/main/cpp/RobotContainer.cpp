// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>

#include <cmath>

RobotContainer::RobotContainer() noexcept
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command; non-owning pointer is passed by value.
  auto requirements = {dynamic_cast<frc2::Subsystem *>(&m_driveSubsystem)};

  // Drive, as commanded by operator joystick controls.
  m_driveCommand = std::make_unique<frc2::RunCommand>(
      [&]() -> void {
        const auto controls = GetDriveTeleopControls();

        m_driveSubsystem.Drive(
            std::get<0>(controls) * physical::kMaxDriveSpeed,
            std::get<1>(controls) * physical::kMaxDriveSpeed,
            std::get<2>(controls) * physical::kMaxTurnRate,
            std::get<3>(controls));
      },
      requirements);

  // Point swerve modules, but do not actually drive.
  m_pointCommand = std::make_unique<frc2::RunCommand>(
      [&]() -> void {
        const auto controls = GetDriveTeleopControls();

        units::angle::radian_t angle{std::atan2(std::get<0>(controls), std::get<1>(controls))};

        // Ingnore return (bool); no need to check that commanded angle has
        // been reached.
        (void)m_driveSubsystem.SetTurningPosition(angle);
      },
      requirements);

  m_autonomousCommand = std::make_unique<ExampleCommand>(&m_driveSubsystem);

  m_zeroCommand = std::make_unique<ZeroCommand>(&m_driveSubsystem);
  m_xsAndOsCommand = std::make_unique<XsAndOsCommand>(&m_driveSubsystem);
  m_squareCommand = std::make_unique<SquareCommand>(&m_driveSubsystem);
  m_spirographCommand = std::make_unique<SpirographCommand>(&m_driveSubsystem);
  m_orbitCommand = std::make_unique<OrbitCommand>(&m_driveSubsystem);
  m_pirouetteCommand = std::make_unique<PirouetteCommand>(&m_driveSubsystem);

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
  return m_autonomousCommand.get();
}

std::tuple<double, double, double, bool> RobotContainer::GetDriveTeleopControls() noexcept
{
  // The robot's frame of reference is the standard unit circle, from
  // trigonometry.  However, the front of the robot is facing along the positve
  // X axis.  This means the poitive Y axis extends outward from the left (or
  // port) side of the robot.  Poitive rotation is counter-clockwise.  On the
  // other hand, as the controller is held, the Y axis is aligned with forward.
  // And, specifically, it is the negative Y axis which extends forward.  So,
  // the robot's X is the controllers inverted Y.  On the controller, the X
  // axis lines up with the robot's Y axis.  And, the controller's positive X
  // extends to the right.  So, the robot's Y is the controller's inverted X.
  // Finally, the other controller joystick is used for commanding rotation and
  // things work out so that this is also an inverted X axis.
  double x = -m_xbox.GetY(frc::GenericHID::JoystickHand::kLeftHand);
  double y = -m_xbox.GetX(frc::GenericHID::JoystickHand::kLeftHand);
  double z = -m_xbox.GetX(frc::GenericHID::JoystickHand::kRightHand);

  // PlayStation controllers seem to do this strange thing with the rotation:
  // double z = -m_xbox.GetTriggerAxis(frc::GenericHID::JoystickHand::kLeftHand);

  // Add some deadzone, so the robot doesn't drive when the joysticks are
  // released and return to "zero".  These implement a continuous deadband, one
  // in which the full range of outputs may be generated, once joysticks move
  // outside the deadband.

  // Also, cube the result, to provide more opertor control.  Just cubing the
  // raw value does a pretty good job with the deadband, but doing both is easy
  // and guarantees no movement in the deadband.  Cubing makes it easier to
  // command smaller/slower movements, while still being able to command full
  // power.
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

  return std::make_tuple(shape(x), shape(y), shape(z), true);
}

void RobotContainer::TestInit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  m_driveSubsystem.TestInit();

  frc::SendableChooser<frc2::Command *> *chooser = m_driveSubsystem.TestModeChooser();

  chooser->SetDefaultOption("Zero", m_zeroCommand.get());
  chooser->AddOption("Xs and Os", m_xsAndOsCommand.get());
  chooser->AddOption("Point", m_pointCommand.get());
  chooser->AddOption("Square", m_squareCommand.get());
  chooser->AddOption("Spirograph", m_spirographCommand.get());
  chooser->AddOption("Orbit", m_orbitCommand.get());
  chooser->AddOption("Pirouette", m_pirouetteCommand.get());
  chooser->AddOption("Drive", m_driveCommand.get());

  frc2::CommandScheduler::GetInstance().Enable();
}

void RobotContainer::TestExit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  m_driveSubsystem.TestExit();
}

void RobotContainer::TestPeriodic() noexcept
{
  m_driveSubsystem.TestPeriodic();
}
