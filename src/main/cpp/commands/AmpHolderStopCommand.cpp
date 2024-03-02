// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpHolderStopCommand.h"



// Called when the command is initially scheduled.
void AmpHolderStopCommand::Initialize() {
  ampSubsystem->StopAmpHolder();
  finished = true;
}

// Called repeatedly when this Command is scheduled to run
void AmpHolderStopCommand::Execute() {}

// Called once the command ends or is interrupted.
void AmpHolderStopCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool AmpHolderStopCommand::IsFinished() {
  return finished;
}
