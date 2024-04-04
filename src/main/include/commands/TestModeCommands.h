// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/sysid/SysIdRoutine.h>

#include "subsystems/DriveSubsystem.h"

#include <memory>

// Home all swerve modules to zero heading.
class ZeroCommand
    : public frc2::CommandHelper<frc2::Command, ZeroCommand>
{
public:
    explicit ZeroCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("Zero"); }

    void Initialize() noexcept override {}

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override {}

    static frc2::CommandPtr ZeroCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<ZeroCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};
};

// Expose turning maximum velocity and acceleration.
class MaxVAndATurningCommand
    : public frc2::CommandHelper<frc2::Command, ZeroCommand>
{
public:
    explicit MaxVAndATurningCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("MaxVAndATurning"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    static frc2::CommandPtr MaxVAndATurningCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<MaxVAndATurningCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};

    unsigned m_iteration{0};
};

// Expose drive maximum velocity and acceleration.
class MaxVAndADriveCommand
    : public frc2::CommandHelper<frc2::Command, ZeroCommand>
{
public:
    explicit MaxVAndADriveCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("MaxVAndADrive"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    static frc2::CommandPtr MaxVAndADriveCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<MaxVAndADriveCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};

    unsigned m_iteration{0};
};

// Alternately command swerve modules to form X and O patterns.
class XsAndOsCommand
    : public frc2::CommandHelper<frc2::Command, XsAndOsCommand>
{
public:
    explicit XsAndOsCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("Xs and Os"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    static frc2::CommandPtr XsAndOsCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<XsAndOsCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};

    unsigned m_iteration{0};
};

// Command swerve modules to slowly rotate together (spin).
class RotateModulesCommand
    : public frc2::CommandHelper<frc2::Command, RotateModulesCommand>
{
public:
    explicit RotateModulesCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("Rotate Modules"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    static frc2::CommandPtr RotateModulesCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<RotateModulesCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};

    unsigned m_iteration{0};
};

// Drive in a square.
class SquareCommand
    : public frc2::CommandHelper<frc2::Command, SquareCommand>
{
public:
    explicit SquareCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("Square"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    static frc2::CommandPtr SquareCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<SquareCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};

    unsigned m_side{0};
};

// Drive in a spirograph pattern.
class SpirographCommand
    : public frc2::CommandHelper<frc2::Command, SpirographCommand>
{
public:
    explicit SpirographCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("Spirograph"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    static frc2::CommandPtr SpirographCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<SpirographCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};

    unsigned m_angle{0};
};

// Drive in an orbit.
class OrbitCommand
    : public frc2::CommandHelper<frc2::Command, OrbitCommand>
{
public:
    explicit OrbitCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("Orbit"); }

    void Initialize() noexcept override {}

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override {}

    static frc2::CommandPtr OrbitCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<OrbitCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};
};

// Drive in a fancy pirouette.
class PirouetteCommand
    : public frc2::CommandHelper<frc2::Command, PirouetteCommand>
{
public:
    explicit PirouetteCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("Pirouette"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    static frc2::CommandPtr PirouetteCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<PirouetteCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};
};

// Excersize theta (spin) controller.
class SpinCommand
    : public frc2::CommandHelper<frc2::Command, SpinCommand>
{
public:
    explicit SpinCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("Spin"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    static frc2::CommandPtr SpinCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<SpinCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};

    unsigned m_angle{0};
    unsigned m_delay{0};
};

// Perform SysId characterization, logging response to voltage step being
// applied to motors.  Do this sequentially over various subsystems/modalities.
class SysIdCommand
    : public frc2::CommandHelper<frc2::Command, SysIdCommand>
{
public:
    explicit SysIdCommand(DriveSubsystem *subsystem) noexcept
        : m_subsystem{subsystem} { SetName("SysId"); }

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    bool IsFinished() noexcept override;

    static frc2::CommandPtr SysIdCommandFactory(DriveSubsystem *subsystem) noexcept
    {
        return frc2::CommandPtr{std::make_unique<SysIdCommand>(subsystem)};
    }

private:
    DriveSubsystem *m_subsystem{nullptr};

    unsigned m_routine{0};
    unsigned m_mode{0};
    unsigned m_stage{0};

    std::unique_ptr<frc2::sysid::SysIdRoutine> m_driveSysId;
    std::unique_ptr<frc2::sysid::SysIdRoutine> m_steerSysId;
    std::unique_ptr<frc2::CommandPtr> m_commandPtr;
};