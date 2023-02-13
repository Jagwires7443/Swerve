// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc/RobotController.h>
#include <frc/trajectory/Trajectory.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/FeederSubsystem.h"
#include "subsystems/Infrastructure.h"
#include "subsystems/ShooterSubsystem.h"

#include <memory>
#include <string_view>

class TimedAutoBase : public frc2::CommandHelper<frc2::CommandBase, TimedAutoBase>
{
protected:
  TimedAutoBase(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter, std::string_view name) noexcept
      : m_drive{drive}, m_feeder{feeder}, m_infrastructure{infrastructure}, m_shooter{shooter} { SetName(name); }

public:
  virtual ~TimedAutoBase() noexcept = default;

  void Initialize() noexcept override;

  void Execute() noexcept override;

  void End(bool interrupted) noexcept override;

  bool IsFinished() noexcept override { return finished_; }

  virtual bool Iteration(const uint counter) noexcept { return true; }

protected:
  DriveSubsystem *const m_drive;
  FeederSubsystem *const m_feeder;
  InfrastructureSubsystem *const m_infrastructure;
  ShooterSubsystem *const m_shooter;

private:
  bool pressure_{false};
  bool finished_{false};
  uint64_t FPGATime_{0};
  uint counter_{0};
};

class OneBallAuto : public TimedAutoBase
{
public:
  OneBallAuto(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter) noexcept
      : TimedAutoBase(drive, feeder, infrastructure, shooter, "OneBallAuto") {}

  bool Iteration(const uint counter) noexcept override;

  static frc2::CommandPtr OneBallAutoCommandFactory(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter) noexcept
  {
    return frc2::CommandPtr{std::make_unique<OneBallAuto>(drive, feeder, infrastructure, shooter)};
  }
};

class TwoBallAuto : public TimedAutoBase
{
public:
  TwoBallAuto(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter) noexcept
      : TimedAutoBase(drive, feeder, infrastructure, shooter, "TwoBallAuto") {}

  bool Iteration(const uint counter) noexcept override;

  static frc2::CommandPtr TwoBallAutoCommandFactory(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter) noexcept
  {
    return frc2::CommandPtr{std::make_unique<TwoBallAuto>(drive, feeder, infrastructure, shooter)};
  }
};

namespace TrajectoryAuto
{
  frc2::CommandPtr TrajectoryAutoCommandFactory(DriveSubsystem *const drive, std::string_view name, frc::Trajectory &trajectory) noexcept;
};
