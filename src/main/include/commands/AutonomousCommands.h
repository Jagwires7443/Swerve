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
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/Infrastructure.h"
#include "subsystems/IntakeSubsystem.h"
#include "RobotContainer.h"

#include <memory>
#include <string_view>

class TimedAutoBase : public frc2::CommandHelper<frc2::Command, TimedAutoBase>
{
protected:
    TimedAutoBase(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake, std::string_view name) noexcept
        : m_drive{drive}, arm_{arm}, IntakeSubsystem_{intake} { SetName(name); }

public:
    virtual ~TimedAutoBase() noexcept = default;

    void Initialize() noexcept override;

    void Execute() noexcept override;

    void End(bool interrupted) noexcept override;

    bool IsFinished() noexcept override { return finished_; }

    virtual bool Iteration(const uint32_t counter) noexcept { return true; }

protected:
    DriveSubsystem *const m_drive;
    ArmSubsystem *const arm_;
    IntakeSubsystem *const IntakeSubsystem_;

private:
    bool pressure_{false};
    bool finished_{false};
    uint64_t FPGATime_{0};
    uint32_t counter_{0};
};

class BasicBack : public TimedAutoBase
{
public:
    BasicBack(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "BasicBack") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr BasicBackCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
    {
        return frc2::CommandPtr{std::make_unique<BasicBack>(drive, arm, intake)};
    }
};

class MidShoot2Speaker : public TimedAutoBase
{
public:
    MidShoot2Speaker(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "MidShoot2Speaker") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr MidShoot2SpeakerCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
    {
        return frc2::CommandPtr{std::make_unique<MidShoot2Speaker>(drive, arm, intake)};
    }
};
/*
class ShootSpeakerTwice : public TimedAutoBase
{
    public:
        ShootSpeakerTwice(DriveSubsystem *const drive, ArmSubsystem *const arm,  IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "ShootSpeaker") {}

     bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr ShootSpeakerCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm,  IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<ShootSpeakerTwice>(drive, arm, intake)};
    }
};
*/

class Shoot2Taxi : public TimedAutoBase
{
public:
    Shoot2Taxi(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "Shoot2Taxi") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr Shoot2TaxiCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<Shoot2Taxi>(drive, arm, intake)};
    }
};

class OpenRightSide : public TimedAutoBase
{
public:
    OpenRightSide(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "OpenRightSide") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr OpenRightSideCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<OpenRightSide>(drive, arm, intake)};
    }
};

class OpenLeftSide : public TimedAutoBase
{
public:
    OpenLeftSide(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "OpenLeftSide") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr OpenLeftSideCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<OpenLeftSide>(drive, arm, intake)};
    }
};

class LeftSide : public TimedAutoBase
{
public:
    LeftSide(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "LeftSide") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr LeftSideCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<LeftSide>(drive, arm, intake)};
    }
};

class test : public TimedAutoBase
{
public:
    test(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "test") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr testCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<test>(drive, arm, intake)};
    }
};
class SideShoot1 : public TimedAutoBase
{
public:
    SideShoot1(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "SideShoot1") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr SideShoot1CommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<SideShoot1>(drive, arm, intake)};
    }
};

class LeftShoot2Speaker : public TimedAutoBase
{
public:
    LeftShoot2Speaker(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "LeftShoot2Speaker") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr LeftShoot2SpeakerCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<LeftShoot2Speaker>(drive, arm, intake)};
    }
};

class RightSide : public TimedAutoBase
{
public:
    RightSide(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept
        : TimedAutoBase(drive, arm, intake, "RightSide") {}

    bool Iteration(const uint32_t counter) noexcept override;

    static frc2::CommandPtr RightSideCommandFactory(DriveSubsystem *const drive, ArmSubsystem *const arm, IntakeSubsystem *const intake) noexcept

    {
        return frc2::CommandPtr{std::make_unique<RightSide>(drive, arm, intake)};
    }
};
