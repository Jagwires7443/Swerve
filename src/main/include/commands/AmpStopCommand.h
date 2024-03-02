// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/AmpSubsystem.h"

/**
 * Stops the Amp Arm while extending or retracting
 */
class AmpStopCommand
    : public frc2::CommandHelper<frc2::Command, AmpStopCommand> {
 public:
  explicit AmpStopCommand(AmpSubsystem *ampSubsystem)
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
