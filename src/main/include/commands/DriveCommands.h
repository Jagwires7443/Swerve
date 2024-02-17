// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>

#include "subsystems/DriveSubsystem.h"

#include <memory>

// Home all swerve modules to zero heading.
class ZeroTurningModules
    : public frc2::CommandHelper<frc2::Command, ZeroTurningModules>
{
public:
    explicit ZeroTurningModules(DriveSubsystem *driveSubsystem) noexcept
        : driveSubsystem{driveSubsystem} { SetName("Zero"); }

    void Initialize() noexcept override {}
    void Execute() noexcept override;
    void End(bool interrupted) noexcept override {}
    bool IsFinished() noexcept override { return finished; }

    static frc2::CommandPtr ZeroCommandFactory(DriveSubsystem *driveSubsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<ZeroTurningModules>(driveSubsystem)};
    }

private:
    DriveSubsystem *driveSubsystem{nullptr};
    bool finished{false};
};

// Expose turning maximum velocity and acceleration.
class DriveCommand
    : public frc2::CommandHelper<frc2::Command, DriveCommand>
{
public:
    explicit DriveCommand(DriveSubsystem *driveSubsystem) noexcept
        : driveSubsystem{driveSubsystem} { SetName("DriveCommand"); }

    void Initialize() noexcept override;
    void Execute() noexcept override;
    void End(bool interrupted) noexcept override;
    bool IsFinished() noexcept override { return finished; }

    static frc2::CommandPtr DriveCommandFactory(DriveSubsystem *driveSubsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<DriveCommand>(driveSubsystem)};
    }

private:
    DriveSubsystem *driveSubsystem{nullptr};
    bool finished{false};
};
