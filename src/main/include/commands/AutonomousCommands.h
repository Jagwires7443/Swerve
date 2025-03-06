// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

// #include <frc2/command/Command.h>
// #include <frc2/command/CommandPtr.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/RobotController.h>
#include <frc/trajectory/Trajectory.h>
//nclude <pathplanner/lib/commands/PathPlannerAuto.h>
//nclude <pathplanner/lib/auto/AutoBuilder.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Infrastructure.h"
#include "subsystems/IntakeSubsystem.h"
#include "RobotContainer.h"

#include <memory>
#include <string_view>


class TimedAutoBase : public frc2::CommandHelper<frc2::Command, TimedAutoBase>
{
protected:
    TimedAutoBase(DriveSubsystem *const drive, IntakeSubsystem *const intake, std::string_view name) noexcept
        : m_drive{drive} { SetName(name); }

public:
    virtual ~TimedAutoBase() noexcept = default;

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    bool IsFinished() noexcept override { return finished_; }

    virtual bool Iteration(const uint32_t counter) noexcept { return true; }

protected:
    DriveSubsystem *const m_drive;

private:
    bool pressure_{false};
    bool finished_{false};
    uint64_t FPGATime_{0};
    uint32_t counter_{0};
};
