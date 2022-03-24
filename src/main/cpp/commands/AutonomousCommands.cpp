// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

AutonomousCommand::AutonomousCommand(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter) noexcept
    : m_drive{drive}, m_feeder{feeder}, m_infrastructure{infrastructure}, m_shooter{shooter} { SetName("Autonomous"); }

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
    uint deltaTime = (FPGATime - FPGATime_) / 1000; // ms

    if (deltaTime < 100) // 100ms
    {
        return;
    }

    FPGATime_ = FPGATime;

    // Will arrive here every 100ms, for 15s autonomous period.

    const units::pressure::pounds_per_square_inch_t pressure = m_infrastructure->GetPressure();

    printf("**** Auto(%u): %lf\n", counter_, pressure.to<double>()); // XXX

    if (pressure < 75_psi)
    {
        return;
    }

    ++counter_;

    if (counter_ <= 5) // 500ms
    {
        // m_drive->Drive();
        m_feeder->DropIntake();
        m_feeder->RaiseIntake();

        return;
    }

    if (counter_ <= 10) // 1s
    {
        m_feeder->LowerIntake();

        return;
    }

    if (counter_ <= 20) // 2s
    {
        m_feeder->LockIntake();

        return;
    }

    finished_ = true;
}

void AutonomousCommand::End(bool interrupted) noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_feeder->Default(0.0);

    m_shooter->Stop();
}
