// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PositionTransferArmCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

void PositionTransferArm::Initialize() noexcept
{
    finished = false;

    // transferArmSubsystem->SetTransferArmPosition(position);
    timer.Reset();
    timer.Start();
}

void PositionTransferArm::Execute() noexcept
{
    frc::SmartDashboard::PutNumber("Arm Timer ", timer.Get().value());
    // transferArmSubsystem->UpdatePIDValues();s
    transferArmSubsystem->SetArmMotorVoltagePercent(.05);
    if (timer.HasElapsed(2_s)) { finished = true; }
}

void PositionTransferArm::End(bool interrupted) noexcept
{
    transferArmSubsystem->StopTransferArm();
}
