// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include "frc/DigitalInput.h"
#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"
#include "subsystems/TransferArmSubsystem.h"

/**
 * This command is for picking up a note and moving to the home position with the arm
 */
class IntakeCommand
    : public frc2::CommandHelper<frc2::Command, IntakeCommand> {
 public:
  explicit IntakeCommand(IntakeSubsystem *intakeSubsystem) 
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
    frc::DigitalInput limit1{8};
    frc::DigitalInput limit2{9};

};
