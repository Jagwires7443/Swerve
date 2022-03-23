// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

AutonomousCommand::AutonomousCommand(DriveSubsystem *const drive, FeederSubsystem *const feeder, ShooterSubsystem *const shooter) noexcept
    : m_drive{drive}, m_feeder{feeder}, m_shooter{shooter} { SetName("Autonomous"); }

void AutonomousCommand::Initialize() noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_feeder->Default(0.0);

    m_shooter->Stop();

    FPGATime_ = frc::RobotController::GetFPGATime();
    counter_ = 0;
}

void AutonomousCommand::Execute() noexcept
{
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    uint deltaTime = FPGATime - FPGATime_; // Microseconds.

    if (deltaTime < 100 * 1000) // 100ms.
    {
        return;
    }

    FPGATime_ = FPGATime;

    // Will arrive here every 100ms, for 15s autonomous period.

    if (counter_ < 5)
    {
        // m_drive->Drive();
        m_feeder->DropIntake();
        m_feeder->RaiseIntake();
    }
    else if (counter_ < 10)
    {
        m_feeder->LowerIntake();
    }
    else if (counter_ < 15)
    {
        m_feeder->LockIntake();
    }

    ++counter_;
}

void AutonomousCommand::End(bool interrupted) noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_feeder->Default(0.0);

    m_shooter->Stop();
}
