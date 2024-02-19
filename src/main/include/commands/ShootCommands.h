// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TransferArmSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootCommands
    : public frc2::CommandHelper<frc2::Command, ShootCommands>
{
public:
  explicit ShootCommands(ShooterSubsystem *shooterSubsystem)
      : shooterSubsystem{shooterSubsystem}
  {
    AddRequirements(shooterSubsystem);
  }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  ShooterSubsystem *shooterSubsystem{nullptr};
  frc::Timer timer{};
  bool finished{false};
};
