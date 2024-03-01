// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/AmpSubsystem.h"
#include <frc/Timer.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class AmpRetractCommand
    : public frc2::CommandHelper<frc2::Command, AmpRetractCommand> {
 public:
  AmpRetractCommand(AmpSubsystem *ampSubsystem)
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
   bool finished = false;
   frc::Timer timer{};
};
