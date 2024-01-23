// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc/RobotController.h>
#include <frc/trajectory/Trajectory.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Infrastructure.h"

#include <memory>
#include <string_view>

namespace TrajectoryAuto
{
//  frc2::CommandPtr TrajectoryAutoCommandFactory(DriveSubsystem *const drive, std::string_view name, frc ::Trajectory &trajectory) noexcept;
}

namespace pathplanner
{
frc2::CommandPtr RobotContainer::GetAutonomousCommand() noexcept
{
  return PathPlannerAuto("examplename").ToPtr();
}
}
