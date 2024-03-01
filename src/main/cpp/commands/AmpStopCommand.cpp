// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpStopCommand.h"


// Called when the command is initially scheduled.
void AmpStopCommand::Initialize() {
  ampSubsystem->StopAmpExtend();
  finished = false;
}

// Called repeatedly when this Command is scheduled to run
void AmpStopCommand::Execute() {}

// Called once the command ends or is interrupted.
void AmpStopCommand::End(bool interrupted) {
  ampSubsystem->StopAmpExtend();
}

// Returns true when the command should end.
bool AmpStopCommand::IsFinished() {
  return finished;
}
