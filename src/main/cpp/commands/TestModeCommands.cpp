// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TestModeCommands.h"

ZeroCommand::ZeroCommand(DriveSubsystem *subsystem) noexcept
    : m_subsystem{subsystem} { SetName("Zero"); }

XsAndOsCommand::XsAndOsCommand(DriveSubsystem *subsystem) noexcept
    : m_subsystem{subsystem} { SetName("Xs and Os"); }

SquareCommand::SquareCommand(DriveSubsystem *subsystem) noexcept
    : m_subsystem{subsystem} { SetName("Square"); }

SpirographCommand::SpirographCommand(DriveSubsystem *subsystem) noexcept
    : m_subsystem{subsystem} { SetName("Spirograph"); }

OrbitCommand::OrbitCommand(DriveSubsystem *subsystem) noexcept
    : m_subsystem{subsystem} { SetName("Orbit"); }

PirouetteCommand::PirouetteCommand(DriveSubsystem *subsystem) noexcept
    : m_subsystem{subsystem} { SetName("Pirouette"); }

void ZeroCommand::Execute() noexcept {}

void XsAndOsCommand::Execute() noexcept {}

void SquareCommand::Execute() noexcept {}

void SpirographCommand::Execute() noexcept {}

void OrbitCommand::Execute() noexcept {}

void PirouetteCommand::Execute() noexcept {}
