// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/ParallelCommandGroup.h>
#include "commands/PositionTransferArmCommand.h"
#include "commands/ShootCommands.h"
#include "commands/IntakeEjectCommand.h"
#include "commands/PIDTransferArmCommand.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/TransferArmSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootProcedureCommand
    : public frc2::CommandHelper<frc2::Command, ShootProcedureCommand> {
 public:
  explicit ShootProcedureCommand(IntakeSubsystem *m_intakeSubsystem, TransferArmSubsystem *m_transferArmSubsystem, ShooterSubsystem *m_shooterSubsystem ) 
      : m_intakeSubsystem{m_intakeSubsystem}, m_transferArmSubsystem{m_transferArmSubsystem}, m_shooterSubsystem{m_shooterSubsystem}
      {
        AddRequirements(m_intakeSubsystem);
        AddRequirements(m_transferArmSubsystem);
        AddRequirements(m_shooterSubsystem);
      }

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  IntakeSubsystem *m_intakeSubsystem{nullptr};
  TransferArmSubsystem *m_transferArmSubsystem{nullptr};
  ShooterSubsystem *m_shooterSubsystem{nullptr};
};
