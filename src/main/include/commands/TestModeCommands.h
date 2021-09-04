// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

// Home all swerve modules to zero heading.
class ZeroCommand
    : public frc2::CommandHelper<frc2::CommandBase, ZeroCommand>
{
public:
    explicit ZeroCommand(DriveSubsystem *subsystem) noexcept;

    void Initialize() noexcept override {}

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override {}

private:
    DriveSubsystem *m_subsystem{nullptr};
};

// Alternately command swerve modules to form X and O patterns.
class XsAndOsCommand
    : public frc2::CommandHelper<frc2::CommandBase, XsAndOsCommand>
{
public:
    explicit XsAndOsCommand(DriveSubsystem *subsystem) noexcept;

    void Initialize() noexcept override {}

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override {}

private:
    DriveSubsystem *m_subsystem{nullptr};
};

// Drive in a square.
class SquareCommand
    : public frc2::CommandHelper<frc2::CommandBase, SquareCommand>
{
public:
    explicit SquareCommand(DriveSubsystem *subsystem) noexcept;

    void Initialize() noexcept override {}

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override {}

private:
    DriveSubsystem *m_subsystem{nullptr};
};

// Drive in a spirograph pattern.
class SpirographCommand
    : public frc2::CommandHelper<frc2::CommandBase, SpirographCommand>
{
public:
    explicit SpirographCommand(DriveSubsystem *subsystem) noexcept;

    void Initialize() noexcept override {}

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override {}

private:
    DriveSubsystem *m_subsystem{nullptr};
};

// Drive in an orbit.
class OrbitCommand
    : public frc2::CommandHelper<frc2::CommandBase, OrbitCommand>
{
public:
    explicit OrbitCommand(DriveSubsystem *subsystem) noexcept;

    void Initialize() noexcept override {}

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override {}

private:
    DriveSubsystem *m_subsystem{nullptr};
};

// Drive in a fancy pirouette.
class PirouetteCommand
    : public frc2::CommandHelper<frc2::CommandBase, PirouetteCommand>
{
public:
    explicit PirouetteCommand(DriveSubsystem *subsystem) noexcept;

    void Initialize() noexcept override {}

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override {}

private:
    DriveSubsystem *m_subsystem{nullptr};
};
