// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveCommands.h"

#include <cmath>

void ZeroTurningModules::Execute() noexcept { (void)driveSubsystem->ZeroModules(); }

void DriveCommand::Initialize() noexcept
{
    driveSubsystem->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, true);
}

void DriveCommand::Execute() noexcept
{
    // TODO: call handle drive subsystem drive
    // this will need to accept a parameter in the constructor
    // and will be implemented similar to RobotContainer.cpp
    // drivecommandfactory.
    // Instead of reading the values from the controller input
    // they will be passed in as parameters.
}

void DriveCommand::End(bool interrupted) noexcept
{
    driveSubsystem->Drive(0.0_mps, 0.0_mps, 0.0_rad_per_s, true);
}
