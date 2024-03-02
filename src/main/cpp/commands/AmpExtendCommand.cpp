// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AmpExtendCommand.h"
#include "Constants.h"


// Called when the command is initially scheduled.
void AmpExtendCommand::Initialize() {
  //Start the motor to extedn the arm and timer
  ampSubsystem->SetAmpExtendMotorVoltagePercent(amp::kAmpExtendMotorVoltagePercent);
  finished = false;
  timer.Reset();
  timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void AmpExtendCommand::Execute() {
  if (timer.HasElapsed(3_s)){
    finished = true;
    ampSubsystem->StopAmpExtend();
  }

}

// Called once the command ends or is interrupted.
void AmpExtendCommand::End(bool interrupted) {
  ampSubsystem->StopAmpExtend();
}

// Returns true when the command should end.
bool AmpExtendCommand::IsFinished() {
  return finished;
}
