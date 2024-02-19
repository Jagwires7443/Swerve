// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this pr
#include "Constants.h"
#include "RobotContainer.h"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include "commands/ShootCommands.h"
#include "commands/PositionTransferArmCommand.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <units/acceleration.h>
#include <units/velocity.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

RobotContainer::RobotContainer() noexcept
{
  // Initialize all of your commands and subsystems here

  m_LEDPatternCount = m_infrastructureSubsystem.GetLEDPatternCount();

  // Configure the button bindings
  ConfigureBindings();
}

#pragma region Autonomous
void RobotContainer::AutonomousInit() noexcept
{
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                      {&m_driveSubsystem}));
  m_intakeSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                       {&m_intakeSubsystem}));
  m_transferArmSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                       {&m_transferArmSubsystem}));
  m_shooterSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                        {&m_shooterSubsystem}));
  m_infrastructureSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                               {&m_infrastructureSubsystem}));
}

void RobotContainer::AutonomousPeriodic() noexcept {}

void RobotContainer::AutonomousExit() noexcept {}

std::optional<frc2::CommandPtr> RobotContainer::GetAutonomousCommand() noexcept
{
  frc::TrajectoryConfig trajectoryConfig{4.0_mps, 2.0_mps_sq};
  frc::SwerveDriveKinematics<4> kinematics{m_driveSubsystem.kDriveKinematics};

  trajectoryConfig.SetKinematics(kinematics);

  // TODO: Update trajectory path to our autonomous path
  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      {frc::Pose2d{},
       frc::Pose2d{1.0_m, 0.0_m, frc::Rotation2d{}}},
      trajectoryConfig);

  return TrajectoryAuto::TrajectoryAutoCommandFactory(&m_driveSubsystem, "Test Trajectory", trajectory);
}
#pragma endregion

#pragma region Teleop
void RobotContainer::TeleopInit() noexcept
{
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();
  m_driveSubsystem.ZeroHeading();

  m_driveSubsystem.SetDefaultCommand(DriveCommandFactory(this));
  m_intakeSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                       {&m_intakeSubsystem}));
  m_shooterSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                        {&m_shooterSubsystem}));
  m_infrastructureSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                               {&m_infrastructureSubsystem}));
}

void RobotContainer::TeleopPeriodic() noexcept {}

void RobotContainer::TeleopExit() noexcept {}

frc2::CommandPtr RobotContainer::DriveCommandFactory(RobotContainer *container) noexcept
{
  // Set up default drive command; non-owning pointer is passed by value.
  auto driveRequirements = {dynamic_cast<frc2::Subsystem *>(&container->m_driveSubsystem)};

  // Drive, as commanded by operator joystick controls.
  return frc2::CommandPtr{std::make_unique<frc2::RunCommand>(
      [container]() -> void
      {
        if (container->m_lock)
        {
          (void)container->m_driveSubsystem.SetLockWheelsX();

          return;
        }

        const auto controls = container->GetDriveTeleopControls();

        container->m_driveSubsystem.Drive(
            std::get<0>(controls) * physical::kMaxDriveSpeed,
            std::get<1>(controls) * physical::kMaxDriveSpeed,
            std::get<2>(controls) * physical::kMaxTurnRate,
            std::get<3>(controls));
      },
      driveRequirements)};
}

std::tuple<double, double, double, bool> RobotContainer::GetDriveTeleopControls() noexcept
{
  /*
  The robot's frame of reference is the standard unit circle, from
  trigonometry.  However, the front of the robot is facing along the positve
  X axis.  This means the poitive Y axis extends outward from the left (or
  port) side of the robot.  Poitive rotation is counter-clockwise.  On the
  other hand, as the controller is held, the Y axis is aligned with forward.
  And, specifically, it is the negative Y axis which extends forward.  So,
  the robot's X is the controllers inverted Y.  On the controller, the X
  axis lines up with the robot's Y axis.  And, the controller's positive X
  extends to the right.  So, the robot's Y is the controller's inverted X.
  Finally, the other controller joystick is used for commanding rotation and
  things work out so that this is also an inverted X axis.
  */
  double LeftStickX = -m_xbox.GetLeftY();
  double LeftStickY = -m_xbox.GetLeftX();
  double rightStickRot = -m_xbox.GetRightX();

  if (triggerSpeedEnabled) // scale speed by analog trigger
  {
    double RightTrigAnalogVal = m_xbox.GetRightTriggerAxis();
    RightTrigAnalogVal = ConditionRawTriggerInput(RightTrigAnalogVal);

    if (LeftStickX != 0 || LeftStickY != 0)
    {
      if (LeftStickX != 0)
      {
        double LeftStickTheta = atan(LeftStickY / LeftStickX);
        LeftStickX = RightTrigAnalogVal * cos(LeftStickTheta);
        LeftStickY = RightTrigAnalogVal * sin(LeftStickTheta);
      }
      else
      {
        LeftStickY = std::copysign(RightTrigAnalogVal, LeftStickY);
      }
    }
  }
  else // scale speed by analog stick
  {
    LeftStickX = ConditionRawJoystickInput(LeftStickX);
    LeftStickY = ConditionRawJoystickInput(LeftStickY);
  }

  rightStickRot = ConditionRawJoystickInput(rightStickRot, 0.0);

  // TODO: decide if this is still needed
  LeftStickX *= 2.0;
  LeftStickY *= 2.0;
  rightStickRot *= 1.6;

  frc::SmartDashboard::PutNumber("X", LeftStickX);
  frc::SmartDashboard::PutNumber("Y", LeftStickY);
  frc::SmartDashboard::PutNumber("Z", rightStickRot);

  return std::make_tuple(LeftStickX, LeftStickY, rightStickRot, m_fieldOriented);
}

double RobotContainer::ConditionRawTriggerInput(double RawTrigVal) noexcept
{
  // Set a raw trigger value using the right trigger
  RawTrigVal = m_xbox.GetRightTriggerAxis();
  double deadZoneVal = 0.05;
  double deadZoneCorrection = 1.0 / (1.0 - deadZoneVal);

  if (RawTrigVal < deadZoneVal)
  {
    /*if the trigger value is less than deadzonevalue, it will
    set the right trigger value to zero to correct drifting*/
    return 0;
  }
  else
  {
    /* if the trigger value is greater than the deadzonevalue, it will
    make a small correction to stop the increase in speed from being
    too big */
    RawTrigVal -= deadZoneVal;
    RawTrigVal *= deadZoneCorrection;
    return std::pow(RawTrigVal, 3.0); // std::pow(RawTrigVal, 3.0) == RawTrigVal^3
  }
}

double RobotContainer::ConditionRawJoystickInput(double RawJoystickVal, double mixer) noexcept
{
  /*
  PlayStation controllers seem to do this strange thing with the rotation:
  double z = -m_xbox.GetLeftTriggerAxis();
  Note: there is now a PS4Controller class.

  Add some deadzone, so the robot doesn't drive when the joysticks are
  released and return to "zero".  These implement a continuous deadband, one
  in which the full range of outputs may be generated, once joysticks move
  outside the deadband.

  Also, cube the result, to provide more opertor control.  Just cubing the
  raw value does a pretty good job with the deadband, but doing both is easy
  and guarantees no movement in the deadband.  Cubing makes it easier to
  command smaller/slower movements, while still being able to command full
  power.  The 'mixer` parameter is used to shape the `raw` input, some mix
  between out = in^3.0 and out = in.
  */

  // Input deadband around 0.0 (+/- range).
  constexpr double deadZoneVal = 0.05;

  constexpr double slope = 1.0 / (1.0 - deadZoneVal);

  if (RawJoystickVal >= -deadZoneVal && RawJoystickVal <= +deadZoneVal)
  {
    RawJoystickVal = 0.0;
  }
  else if (RawJoystickVal < -deadZoneVal)
  {
    RawJoystickVal += deadZoneVal;
    RawJoystickVal *= slope;
  }
  else if (RawJoystickVal > +deadZoneVal)
  {
    RawJoystickVal -= deadZoneVal;
    RawJoystickVal *= slope;
  }

  return mixer * std::pow(RawJoystickVal, 3.0) + (1.0 - mixer) * RawJoystickVal;
}

void RobotContainer::ConfigureBindings() noexcept
{
  // TODO: define Keybindings here
  m_xbox.Start().OnTrue(
      frc2::InstantCommand([&]() -> void
                           { triggerSpeedEnabled = !triggerSpeedEnabled; },
                           {})
          .ToPtr());

  // TODO: decide if we want this
  // m_xbox.X().OnTrue(frc2::InstantCommand([&]() -> void
  //                                        { m_fieldOriented = false; },
  //                                        {})
  //                       .ToPtr());
  // m_xbox.Y().OnTrue(frc2::InstantCommand([&]() -> void
  //                                        { m_driveSubsystem.ZeroHeading();
  //                                          m_fieldOriented = true; },
  //                                        {&m_driveSubsystem})
  //                       .ToPtr());

  // TODO: decide if we want to bind wheel lock

  m_xbox.A().OnTrue(frc2::InstantCommand([&]() -> void
                                         { m_intakeSubsystem.SetSpinMotorVoltagePercent(intake::kIntakeSpinMotorVoltagePercent); },
                                         {&m_intakeSubsystem})
                        .ToPtr());
  m_xbox.A().OnFalse(frc2::InstantCommand([&]() -> void
                                          { m_intakeSubsystem.StopIntake(); },
                                          {&m_intakeSubsystem})
                         .ToPtr());

  m_xbox.B().OnTrue(frc2::InstantCommand([&]() -> void
                                         { m_shooterSubsystem.SetShooterMotorVoltagePercent(shooter::kShooterMotorVoltagePercent); },
                                         {&m_shooterSubsystem})
                        .ToPtr());
  m_xbox.B().OnFalse(frc2::InstantCommand([&]() -> void
                                          { m_shooterSubsystem.StopShooter(); },
                                          {&m_shooterSubsystem})
                         .ToPtr());
  m_xbox.X().OnTrue(frc2::InstantCommand.&m_shooterSubsystem}
                                          {ShootCommands[&]}
                        .ToPtr ());
 
                                  

}
#pragma endregion

#pragma region Test
void RobotContainer::TestInit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.TestInit();

  frc::SendableChooser<std::function<frc2::CommandPtr()>> *chooser{m_driveSubsystem.TestModeChooser()};

  frc2::CommandScheduler::GetInstance().Enable();
}

void RobotContainer::TestPeriodic() noexcept
{
  m_driveSubsystem.TestPeriodic();
}

void RobotContainer::TestExit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.TestExit();

  m_driveSubsystem.BurnConfig();
}

frc2::CommandPtr RobotContainer::PointCommandFactory(RobotContainer *container) noexcept
{
  // Set up default drive command; non-owning pointer is passed by value.
  auto driveRequirements = {dynamic_cast<frc2::Subsystem *>(&container->m_driveSubsystem)};

  // Point swerve modules, but do not actually drive.
  return frc2::CommandPtr{std::make_unique<frc2::RunCommand>(
      [container]() -> void
      {
        const auto controls = container->GetDriveTeleopControls();

        units::angle::radian_t angle{std::atan2(std::get<0>(controls), std::get<1>(controls))};

        // Ingnore return (bool); no need to check that commanded angle has
        // been reached.
        (void)container->m_driveSubsystem.SetTurningPosition(angle);
      },
      driveRequirements)};
}
#pragma endregion

#pragma region Disabled
void RobotContainer::DisabledInit() noexcept
{
  frc2::CommandScheduler::GetInstance().CancelAll();

  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  // Useful things may be done disabled... (construct, config, dashboard, etc.)
  frc2::CommandScheduler::GetInstance().Enable();

  m_driveSubsystem.DisabledInit();
}

void RobotContainer::DisabledPeriodic() noexcept {}

void RobotContainer::DisabledExit() noexcept
{
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.DisabledExit();
}
#pragma endregion
