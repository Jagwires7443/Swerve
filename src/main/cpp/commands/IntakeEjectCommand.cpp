// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeEjectCommand.h"


// Called when the command is initially scheduled.
void IntakeEjectCommand::Initialize() {
  /* Start the intake motors in the reverse direction
     Set finished to false to allow multiple calls of this command
  */
  
  
  finished = false;
  timer.Reset();
  timer.Start();
  
}

// Called repeatedly when this Command is scheduled to run
void IntakeEjectCommand::Execute() {
  
  if (timer.HasElapsed(3.0_s)){
    intakeSubsystem->SetSpinMotorVoltagePercent(intake::kIntakeSpinMotorEjectVoltagePercent);
  }

  //Run the intake motors in reverse for 2 seconds then stop the intake
  if (timer.HasElapsed(5_s)){
    finished = true;
    intakeSubsystem->StopIntake();
  }
}

// Called once the command ends or is interrupted.
void IntakeEjectCommand::End(bool interrupted) {
  //Stop the intake motors
  intakeSubsystem->StopIntake();
}

// Returns true when the command should end.
bool IntakeEjectCommand::IsFinished() {
  return finished;
}
