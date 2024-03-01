// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/AmpSubsystem.h"

/**
 * This command is to stop the Holder motors from running
 * 
 */
class AmpHolderStopCommand
    : public frc2::CommandHelper<frc2::Command, AmpHolderStopCommand> {
 public:
  explicit AmpHolderStopCommand(AmpSubsystem *ampSubsystem)
      : ampSubsystem{ampSubsystem}
      {
        AddRequirements(ampSubsystem);
      }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

private:
  AmpSubsystem *ampSubsystem{nullptr};
  bool finished{false};
};
