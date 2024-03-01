// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommands.h"

void ZeroTurningModules::Execute() noexcept { (void)driveSubsystem->ZeroModules(); }

void DriveCommand::Initialize() noexcept
{
    timer.Reset();
    timer.Start();
    finished = false;
    driveSubsystem->Drive(
    xspeed * physical::kMaxDriveSpeed,
    yspeed * physical::kMaxDriveSpeed,
    rotation * physical::kMaxTurnRate,
    true);
}

void DriveCommand::Execute() noexcept
{
    if (timer.HasElapsed(time)) { finished = true; }
}

void DriveCommand::End(bool interrupted) noexcept
{
    driveSubsystem->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, true);
}
