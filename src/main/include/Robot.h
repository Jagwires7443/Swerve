// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() noexcept override;
  void RobotPeriodic() noexcept override;
  void DisabledInit() noexcept override;
  void DisabledPeriodic() noexcept override;
  void DisabledExit() noexcept override;
  void AutonomousInit() noexcept override;
  void AutonomousPeriodic() noexcept override;
  void AutonomousExit() noexcept override;
  void TeleopInit() noexcept override;
  void TeleopPeriodic() noexcept override;
  void TeleopExit() noexcept override;
  void TestInit() noexcept override;
  void TestPeriodic() noexcept override;
  void TestExit() noexcept override;

private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  RobotContainer m_container;
};
