// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpHolderGrabCommand.h"
#include "Constants.h"

// Called when the command is initially scheduled.
void AmpHolderGrabCommand::Initialize() {

  ampSubsystem->SetAmpHolderMotorVoltagePercent(amp::kAmpHolderMotorVoltagePercent);
  finished = false;
  timer.Reset();
  timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void AmpHolderGrabCommand::Execute() {
  if (timer.HasElapsed(2_s)){
    finished = true; 
    ampSubsystem->StopAmpHolder();
  }
}

// Called once the command ends or is interrupted.
void AmpHolderGrabCommand::End(bool interrupted) {
  ampSubsystem->StopAmpHolder();
}

// Returns true when the command should end.
bool AmpHolderGrabCommand::IsFinished() {
  return finished;
}
