// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootProcedureCommand.h"

/*
ShootProcedureCommand::ShootProcedureCommand() {
  // Use addRequirements() here to declare subsystem dependencies.
}*/

// Called when the command is initially scheduled.
void ShootProcedureCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ShootProcedureCommand::Execute() {

  frc2::SequentialCommandGroup{
    PIDPositionTransferArm(0_deg, &m_transferArmSubsystem),
      frc2::ParallelCommandGroup{ShootCommands(&m_shooterSubsystem),IntakeEjectCommand(&m_intakeSubsystem)},
  };

}

// Called once the command ends or is interrupted.
void ShootProcedureCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ShootProcedureCommand::IsFinished() {
  return false;
}
