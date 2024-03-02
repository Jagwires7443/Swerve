// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpRetractCommand.h"
#include "Constants.h"

// Called when the command is initially scheduled.
void AmpRetractCommand::Initialize() {

  ampSubsystem->SetAmpExtendMotorVoltagePercent(amp::kAmpRetractMotorVoltagePercent);
  finished = false;
  timer.Reset();
  timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void AmpRetractCommand::Execute() {
  if (timer.HasElapsed(3_s)){
    finished = true;
    ampSubsystem->StopAmpExtend();
  }
}

// Called once the command ends or is interrupted.
void AmpRetractCommand::End(bool interrupted) {
  ampSubsystem->StopAmpExtend();
}

// Returns true when the command should end.
bool AmpRetractCommand::IsFinished() {
  return finished;
}
