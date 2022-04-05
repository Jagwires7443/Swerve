// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/Infrastructure.h"
#include "subsystems/ShooterSubsystem.h"

#include <frc/RobotController.h>

class OneBallAuto
    : public frc2::CommandHelper<frc2::CommandBase, OneBallAuto>
{
public:
  OneBallAuto(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter) noexcept;

  void Initialize() noexcept override;

  void Execute() noexcept override;

  void End(bool interrupted) noexcept override;

  bool IsFinished() noexcept override { return finished_; }

private:
  DriveSubsystem *const m_drive;
  FeederSubsystem *const m_feeder;
  InfrastructureSubsystem *const m_infrastructure;
  ShooterSubsystem *const m_shooter;

  bool pressure_{false};
  bool finished_{false};
  uint64_t FPGATime_{0};
  uint counter_{0};
};

class TwoBallAuto
    : public frc2::CommandHelper<frc2::CommandBase, TwoBallAuto>
{
public:
  TwoBallAuto(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter) noexcept;

  void Initialize() noexcept override;

  void Execute() noexcept override;

  void End(bool interrupted) noexcept override;

  bool IsFinished() noexcept override { return finished_; }

private:
  DriveSubsystem *const m_drive;
  FeederSubsystem *const m_feeder;
  InfrastructureSubsystem *const m_infrastructure;
  ShooterSubsystem *const m_shooter;

  bool pressure_{false};
  bool finished_{false};
  uint64_t FPGATime_{0};
  uint counter_{0};
};
