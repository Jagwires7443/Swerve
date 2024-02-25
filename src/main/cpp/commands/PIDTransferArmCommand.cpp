// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PIDTransferArmCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

PIDPositionTransferArm::PIDPositionTransferArm(units::turn_t targetPosition, TransferArmSubsystem* transferArmSubsystem)
    : CommandHelper{frc::PIDController{arm::kArmPositionP, 0, arm::kArmPositionD},
                    // Close loop on heading
                    [transferArmSubsystem]
                    {
                        frc::SmartDashboard::PutNumber("Arm Position ", transferArmSubsystem->GetTransferArmPosition().value());
                        return transferArmSubsystem->GetTransferArmPosition().value();
                    },
                    // Set reference to targetPosition
                    targetPosition.value(),
                    // Pipe output to turn transfer arm
                    [transferArmSubsystem](double output)
                    {
                        frc::SmartDashboard::PutNumber("Arm PID Output ", output);
                        transferArmSubsystem->SetArmMotorVoltagePercent(output);
                    },
                    // Require the transfer arm
                    {transferArmSubsystem}} {
  // Set the controller to be continuous (because it is an angle controller)
  m_controller.EnableContinuousInput(-1, 1);
  // Set the controller tolerance - the delta tolerance ensures the robot is
  // stationary at the setpoint before it is considered as having reached the
  // reference
  m_controller.SetTolerance(.01, .01);

  AddRequirements(transferArmSubsystem);
}

bool PIDPositionTransferArm::IsFinished() {
  return m_controller.AtSetpoint();
}
