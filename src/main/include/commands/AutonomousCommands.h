// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

#include <frc/RobotController.h>

class AutonomousCommand
    : public frc2::CommandHelper<frc2::CommandBase, AutonomousCommand>
{
public:
  AutonomousCommand(DriveSubsystem *const drive, FeederSubsystem *const feeder, ShooterSubsystem *const shooter) noexcept;

  void Initialize() noexcept override;

  void Execute() noexcept override;

  void End(bool interrupted) noexcept override;

private:
  DriveSubsystem *const m_drive;
  FeederSubsystem *const m_feeder;
  ShooterSubsystem *const m_shooter;

  uint64_t FPGATime_{0};
  uint counter_{0};
};
