// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() noexcept
{
  frc::LiveWindow::SetEnabled(false);
  frc::LiveWindow::DisableAllTelemetry();

  // frc::DataLogManager::Start();
  // frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
// This performs the sense portion of sense/think/act, including sending test
// mode telemetry.  It also handles think and act, except in test mode.  In the
// latter case, TestPeriodic() handles manually driven act.
void Robot::RobotPeriodic() noexcept
{
  frc2::CommandScheduler::GetInstance().Run();
  m_container.Periodic();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() noexcept
{
  m_container.TestExit();

  m_container.DisabledInit();
}

void Robot::DisabledPeriodic() noexcept {}

void Robot::DisabledExit() noexcept
{
  m_container.DisabledExit();
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() noexcept
{
  m_container.TestExit();

  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand)
  {
    m_autonomousCommand->Schedule();
  }

  m_container.AutonomousInit();
}

void Robot::AutonomousPeriodic() noexcept {}

void Robot::AutonomousExit() noexcept {}

void Robot::TeleopInit() noexcept
{
  m_container.TestExit();

  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if (m_autonomousCommand)
  {
    m_autonomousCommand->Cancel();
    m_autonomousCommand.reset();
  }

  m_container.TeleopInit();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() noexcept {}

void Robot::TeleopExit() noexcept {}

void Robot::TestInit() noexcept
{
  m_container.TestInit();
}

/**
 * This function is called periodically during test mode.
 * Note that test mode does not follow the command-based paradigm; it follows
 * Init/Periodic.
 */
void Robot::TestPeriodic() noexcept
{
  m_container.TestPeriodic();
}

void Robot::TestExit() noexcept
{
  m_container.TestExit();
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
