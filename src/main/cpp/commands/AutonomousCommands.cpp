// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"

ExampleCommand::ExampleCommand(DriveSubsystem *subsystem) noexcept
    : m_subsystem{subsystem} { SetName("Example"); }

void ExampleCommand::Initialize() noexcept {}

void ExampleCommand::Execute() noexcept {}

void ExampleCommand::End(bool interrupted) noexcept {}
