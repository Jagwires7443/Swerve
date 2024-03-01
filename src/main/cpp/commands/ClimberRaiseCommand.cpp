// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimberRaiseCommand.h"


// Called when the command is initially scheduled.
void ClimberRaiseCommand::Initialize() {
  timer.Reset();
  timer.Start();
  finished = false;
  climberSubsystem->SolenoidUp();
}

// Called repeatedly when this Command is scheduled to run
void ClimberRaiseCommand::Execute() {
  //Start the climber motor to move climbing hook up. 
  climberSubsystem->SetClimberMotorVoltagePercent(climber::kClimberMotorRaiseVoltagePercent);
  // End command after time defined in constants file.
  if (timer.HasElapsed(climber::kClimberRaiseTimer)) { finished = true; }
}

// Called once the command ends or is interrupted.
void ClimberRaiseCommand::End(bool interrupted) {
    climberSubsystem->StopClimber();
    climberSubsystem->SolenoidDown();
}

// Returns true when the command should end.
bool ClimberRaiseCommand::IsFinished() {
  return finished;
}
