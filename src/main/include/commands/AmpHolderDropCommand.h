// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/AmpSubsystem.h"

/**
 * This command pulls the note from the intake into the holder.  
 * Will need to be tied with the intakeejectcommand
 * 
 * Upgrades - check position of arm to ensure it is in the correct position
 */
class AmpHolderDropCommand
    : public frc2::CommandHelper<frc2::Command, AmpHolderDropCommand> {
 public:
  AmpHolderDropCommand(AmpSubsystem *ampSubsystem)
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
   frc::Timer timer{};
};
