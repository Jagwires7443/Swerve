// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/PositionTransferArmCommand.h"
#include <frc/smartdashboard/SmartDashboard.h>

#include <cmath>

void PositionTransferArm::Initialize() noexcept
{
    transferArmSubsystem->SetTransferArmPosition(position);
    timer.Reset();
}

void PositionTransferArm::Execute() noexcept
{
    transferArmSubsystem->UpdatePIDValues();
    if (timer.HasElapsed(5_s)) { finished = true; }
}

void PositionTransferArm::End(bool interrupted) noexcept
{

}
