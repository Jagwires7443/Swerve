// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc/RobotController.h>
#include <frc/trajectory/Trajectory.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/Infrastructure.h"
#include "subsystems/ShooterSubsystem.h"

#include <memory>
#include <string_view>

class TimedAutoBase : public frc2::CommandHelper<frc2::Command, TimedAutoBase>
{
protected:
  TimedAutoBase(DriveSubsystem *const drive, IntakeSubsystem *const Intake, ShooterSubsystem *const shooter, std::string_view name) noexcept
      : m_drive{drive}, m_Intake{Intake}, m_shooter{shooter} { SetName(name); }

public:
  virtual ~TimedAutoBase() noexcept = default;

  void Initialize() noexcept override;

  void Execute() noexcept override;

  void End(bool interrupted) noexcept override;

  bool IsFinished() noexcept override { return finished_; }

  virtual bool Iteration(const uint counter) noexcept { return true; }

protected:
  DriveSubsystem *const m_drive;
  IntakeSubsystem *const m_Intake;
  ShooterSubsystem *const m_shooter;

private:
  bool finished_{false};
  uint64_t FPGATime_{0};
  uint counter_{0};
};

namespace TrajectoryAuto
{
  frc2::CommandPtr TrajectoryAutoCommandFactory(DriveSubsystem *const drive, std::string_view name, frc::Trajectory &trajectory) noexcept;
};
