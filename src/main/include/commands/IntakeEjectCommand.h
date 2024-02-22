// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include <frc/Timer.h>



/**
 * This command is for ejecting the note from the intake.  
 * This can be called by the shootcommand to feed into the shooter 
 * Or used to eject a note onto the field
 */
class IntakeEjectCommand
    : public frc2::CommandHelper<frc2::Command, IntakeEjectCommand> {
 public:
  explicit IntakeEjectCommand(IntakeSubsystem *intakeSubsystem)
      : intakeSubsystem{intakeSubsystem}
      {
        AddRequirements(intakeSubsystem);
      }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
 
 private: 
    IntakeSubsystem *intakeSubsystem{nullptr};
    bool finished{false};
    frc::Timer timer{};
};
