// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"
// #include "RobotContainer.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <units/angle.h>

void TimedAutoBase::Initialize() noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
    m_drive->ResetDrive();
    m_drive->ZeroHeading();

    finished_ = false;
    FPGATime_ = frc::RobotController::GetFPGATime();
    counter_ = 0;
}

void TimedAutoBase::End(bool interrupted) noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

}

void TimedAutoBase::Execute() noexcept
{
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    const uint32_t deltaTime = (FPGATime - FPGATime_) / 1000; // ms

    if (deltaTime < 100) // 100ms
    {
        return;
    }

    if (Iteration(++counter_))
    {

        finished_ = true;
    }
}

