// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/AmpSubsystem.h"
#include <frc/Timer.h>

/**
 * This command is to extend the Amp mechanism for droping in the AMP
 */
class AmpExtendCommand
    : public frc2::CommandHelper<frc2::Command, AmpExtendCommand> {
 public:
  AmpExtendCommand();

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  private:
   AmpSubsystem *ampSubsystem{nullptr};
   bool finished{false};
   frc::Timer timer{}; 

};
