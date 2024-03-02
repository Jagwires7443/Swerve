// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
#include <frc/Timer.h>

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ClimberRaiseCommand
    : public frc2::CommandHelper<frc2::Command, ClimberRaiseCommand> {
 public:
  explicit ClimberRaiseCommand(ClimberSubsystem *climberSubsystem)
      : climberSubsystem{climberSubsystem}
  {
    AddRequirements(climberSubsystem);
  }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  private:
    ClimberSubsystem *climberSubsystem{nullptr};
    frc::Timer timer{};
    frc::Timer solenoidTimer{};
    bool finished{false};
};
