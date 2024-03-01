// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include "subsystems/DriveSubsystem.h"

// Home all swerve modules to zero heading.
class ZeroTurningModules
    : public frc2::CommandHelper<frc2::Command, ZeroTurningModules>
{
public:
    explicit ZeroTurningModules(DriveSubsystem *driveSubsystem) noexcept
        : driveSubsystem{driveSubsystem}
    {
        SetName("Zero");
        AddRequirements(driveSubsystem);
    }

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

// TODO: decide what inputs this will accept
class DriveCommand
    : public frc2::CommandHelper<frc2::Command, DriveCommand>
{
public:
    explicit DriveCommand(float xspeed, float yspeed, float rotation, units::second_t time, DriveSubsystem *driveSubsystem) noexcept
        : driveSubsystem{driveSubsystem},
        xspeed{xspeed},
        yspeed{yspeed},
        rotation{rotation},
        time{time}
    {
        SetName("DriveCommand");
        AddRequirements(driveSubsystem);
    }

    void Initialize() noexcept override;
    void Execute() noexcept override;
    void End(bool interrupted) noexcept override;
    bool IsFinished() noexcept override { return finished; }

private:
    float xspeed, yspeed, rotation;
    units::second_t time;
    DriveSubsystem *driveSubsystem{nullptr};
    frc::Timer timer{};
    bool finished{false};
};
