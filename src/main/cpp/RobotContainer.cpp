// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <cmath>

RobotContainer::RobotContainer() noexcept
{
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command; non-owning pointer is passed by value.
  auto driveRequirements = {dynamic_cast<frc2::Subsystem *>(&m_driveSubsystem)};

  // Drive, as commanded by operator joystick controls.
  m_driveCommand = std::make_unique<frc2::RunCommand>(
      [&]() -> void
      {
        const auto controls = GetDriveTeleopControls();

        m_driveSubsystem.Drive(
            std::get<0>(controls) * physical::kMaxDriveSpeed,
            std::get<1>(controls) * physical::kMaxDriveSpeed,
            std::get<2>(controls) * physical::kMaxTurnRate,
            std::get<3>(controls));
      },
      driveRequirements);

  // Point swerve modules, but do not actually drive.
  m_pointCommand = std::make_unique<frc2::RunCommand>(
      [&]() -> void
      {
        const auto controls = GetDriveTeleopControls();

        units::angle::radian_t angle{std::atan2(std::get<0>(controls), std::get<1>(controls))};

        // Ingnore return (bool); no need to check that commanded angle has
        // been reached.
        (void)m_driveSubsystem.SetTurningPosition(angle);
      },
      driveRequirements);

  m_autonomousCommand = std::make_unique<AutonomousCommand>(&m_driveSubsystem, &m_feederSubsystem, &m_infrastructureSubsystem, &m_shooterSubsystem);

  m_zeroCommand = std::make_unique<ZeroCommand>(&m_driveSubsystem);
  m_maxVAndATurningCommand = std::make_unique<MaxVAndATurningCommand>(&m_driveSubsystem);
  m_maxVAndADriveCommand = std::make_unique<MaxVAndADriveCommand>(&m_driveSubsystem);
  m_xsAndOsCommand = std::make_unique<XsAndOsCommand>(&m_driveSubsystem);
  m_squareCommand = std::make_unique<SquareCommand>(&m_driveSubsystem);
  m_spirographCommand = std::make_unique<SpirographCommand>(&m_driveSubsystem);
  m_orbitCommand = std::make_unique<OrbitCommand>(&m_driveSubsystem);
  m_pirouetteCommand = std::make_unique<PirouetteCommand>(&m_driveSubsystem);
}

void RobotContainer::AutonomousInit() noexcept
{
  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                      {&m_driveSubsystem}));
  m_feederSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                       {&m_feederSubsystem}));
  m_shooterSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                        {&m_shooterSubsystem}));
}

void RobotContainer::TeleopInit() noexcept
{
  m_driveSubsystem.SetDefaultCommand(*m_driveCommand);
  m_feederSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void
                                                       { m_feederSubsystem.Default(m_xbox.GetRightTriggerAxis()); },
                                                       {&m_feederSubsystem}));
  m_shooterSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void
                                                        { m_shooterSubsystem.Default(m_xbox.GetLeftTriggerAxis(), m_shooterVelocity); },
                                                        {&m_shooterSubsystem}));
}

void RobotContainer::ConfigureButtonBindings() noexcept
{
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kA).WhenHeld(frc2::InstantCommand([&]() -> void
                                                                                               { m_slow = true; },
                                                                                               {}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kB).WhenHeld(frc2::InstantCommand([&]() -> void
                                                                                               { m_slow = false; },
                                                                                               {}));

  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kX).WhenHeld(frc2::InstantCommand([&]() -> void
                                                                                               { m_fieldOriented = false; },
                                                                                               {}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kY).WhenHeld(frc2::InstantCommand([&]() -> void
                                                                                               { m_driveSubsystem.ZeroHeading();
                                                                                            m_fieldOriented = true; },
                                                                                               {&m_driveSubsystem}));

  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kLeftBumper).WhileHeld(frc2::InstantCommand([&]() -> void
                                                                                                         { m_feederSubsystem.Fire(); },
                                                                                                         {&m_feederSubsystem}));

  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kRightBumper).WhileHeld(frc2::InstantCommand([&]() -> void
                                                                                                          { m_feederSubsystem.Eject(); },
                                                                                                          {&m_feederSubsystem}));

  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kStart).WhileHeld(frc2::InstantCommand([&]() -> void
                                                                                                    { m_feederSubsystem.Raise(); },
                                                                                                    {&m_feederSubsystem}));

  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kBack).WhileHeld(frc2::InstantCommand([&]() -> void
                                                                                                   { m_feederSubsystem.Lower(); },
                                                                                                   {&m_feederSubsystem}));

  frc2::POVButton(&m_xbox, 90).WhileHeld(frc2::InstantCommand([&]() -> void
                                                              { m_feederSubsystem.LockIntake(); },
                                                              {&m_feederSubsystem}));

  frc2::POVButton(&m_xbox, 270).WhileHeld(frc2::InstantCommand([&]() -> void
                                                               { m_feederSubsystem.DropIntake(); },
                                                               {&m_feederSubsystem}));

  frc2::POVButton(&m_xbox, 0).WhileHeld(frc2::InstantCommand([&]() -> void
                                                             { m_feederSubsystem.RaiseIntake(); },
                                                             {&m_feederSubsystem}));

  frc2::POVButton(&m_xbox, 180).WhileHeld(frc2::InstantCommand([&]() -> void
                                                               { m_feederSubsystem.LowerIntake(); },
                                                               {&m_feederSubsystem}));

  frc2::JoystickButton(&m_buttonBoard, 5).WhenHeld(frc2::InstantCommand([&]() -> void
                                                                        { m_shooterVelocity = -500.0; },
                                                                        {}));

  frc2::JoystickButton(&m_buttonBoard, 6).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                           { m_turbo = true; },
                                                                           {}));

  frc2::JoystickButton(&m_buttonBoard, 6).WhenReleased(frc2::InstantCommand([&]() -> void
                                                                            { m_turbo = false; },
                                                                            {}));

  frc2::JoystickButton(&m_buttonBoard, 10).WhenHeld(frc2::InstantCommand([&]() -> void
                                                                         { m_shooterVelocity = 1200.0; },
                                                                         {}));

  frc2::JoystickButton(&m_buttonBoard, 11).WhenHeld(frc2::InstantCommand([&]() -> void
                                                                         { m_shooterVelocity = 1000.0; },
                                                                         {}));

  frc2::JoystickButton(&m_buttonBoard, 12).WhenHeld(frc2::InstantCommand([&]() -> void
                                                                         { m_shooterVelocity = 300.0; },
                                                                         {}));
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
  double x = -m_xbox.GetLeftY();
  double y = -m_xbox.GetLeftX();
  double z = -m_xbox.GetRightX();

  // PlayStation controllers seem to do this strange thing with the rotation:
  // double z = -m_xbox.GetLeftTriggerAxis();
  // Note: there is now a PS4Controller class.

  // Add some deadzone, so the robot doesn't drive when the joysticks are
  // released and return to "zero".  These implement a continuous deadband, one
  // in which the full range of outputs may be generated, once joysticks move
  // outside the deadband.

  // Also, cube the result, to provide more opertor control.  Just cubing the
  // raw value does a pretty good job with the deadband, but doing both is easy
  // and guarantees no movement in the deadband.  Cubing makes it easier to
  // command smaller/slower movements, while still being able to command full
  // power.  The 'mixer` parameter is used to shape the `raw` input, some mix
  // between out = in^3.0 and out = in.
  auto shape = [](double raw, double mixer = 0.75) -> double
  {
    // Input deadband around 0.0 (+/- range).
    constexpr double range = 0.05;

    constexpr double slope = 1.0 / (1.0 - range);

    if (raw >= -range && raw <= +range)
    {
      raw = 0.0;
    }
    else if (raw < -range)
    {
      raw += range;
      raw *= slope;
    }
    else if (raw > +range)
    {
      raw -= range;
      raw *= slope;
    }

    return mixer * std::pow(raw, 3.0) + (1.0 - mixer) * raw;
  };

  x = shape(x);
  y = shape(y);
  z = shape(z, 0.0);

  if (m_turbo)
  {
    x *= 13.87;
    y *= 13.87;
    z *= 13.87;
  }
  else if (m_slow)
  {
    x *= 0.2;
    y *= 0.2;
    z *= 0.1;
  }

  return std::make_tuple(x, y, z, m_fieldOriented);
}

void RobotContainer::TestInit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  m_driveSubsystem.TestInit();
  m_feederSubsystem.TestInit();
  m_shooterSubsystem.TestInit();

  frc::SendableChooser<frc2::Command *> *chooser = m_driveSubsystem.TestModeChooser();

  chooser->SetDefaultOption("Zero", m_zeroCommand.get());
  chooser->AddOption("Turning Max", m_maxVAndATurningCommand.get());
  chooser->AddOption("Drive Max", m_maxVAndADriveCommand.get());
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
  m_feederSubsystem.TestExit();
  m_shooterSubsystem.TestExit();
}

void RobotContainer::TestPeriodic() noexcept
{
  m_driveSubsystem.TestPeriodic();
  m_feederSubsystem.TestPeriodic();
  m_shooterSubsystem.TestPeriodic();
}

void RobotContainer::DisabledInit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  // Useful things may be done disabled... (construct, config, dashboard, etc.)
  frc2::CommandScheduler::GetInstance().Enable();
}
