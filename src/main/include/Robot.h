// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() noexcept override;
  void RobotPeriodic() noexcept override;
  void DisabledInit() noexcept override;
  void DisabledPeriodic() noexcept override;
  void AutonomousInit() noexcept override;
  void AutonomousPeriodic() noexcept override;
  void TeleopInit() noexcept override;
  void TeleopPeriodic() noexcept override;
  void TestInit() noexcept override;
  void TestPeriodic() noexcept override;

private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command *m_autonomousCommand = nullptr;

  RobotContainer m_container;
};
