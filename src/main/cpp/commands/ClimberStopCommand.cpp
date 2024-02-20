// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberStopCommand.h"


// Called when the command is initially scheduled.
void ClimberStopCommand::Initialize() {}

// Called repeatedly when this Command is scheduled to run
void ClimberStopCommand::Execute() {
  //Stop the motors
  climberSubsystem->StopClimber();
}

// Called once the command ends or is interrupted.
void ClimberStopCommand::End(bool interrupted) {}

// Returns true when the command should end.
bool ClimberStopCommand::IsFinished() {
  return false;
}
