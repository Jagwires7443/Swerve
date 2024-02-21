// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootCommands.h"

// Called when the command is initially scheduled.
void ShootCommands::Initialize() {

  //Start the shooter motors and timer
  shooterSubsystem->SetShooterMotorVoltagePercent(shooter::kShooterMotorVoltagePercent);
  finished = false;
  timer.Reset();
  timer.Start();
}

// Called repeatedly when this Command is scheduled to run
void ShootCommands::Execute() {
  //When the timer has run for 5 seconds set finished to true to stop the motors
  if (timer.HasElapsed(5_s)) {
    finished = true;
    shooterSubsystem->StopShooter();
    }
}

// Called once the command ends or is interrupted.
void ShootCommands::End(bool interrupted) {
  //Stop the shooter motors
  shooterSubsystem->StopShooter();
}

// Returns true when the command should end.
bool ShootCommands::IsFinished() {
  return finished;
}
