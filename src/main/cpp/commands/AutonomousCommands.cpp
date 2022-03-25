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

    m_feeder->LockIntake();
    m_feeder->RaiseIntake();
    m_feeder->Default(0.0);

    m_shooter->Stop();

    FPGATime_ = frc::RobotController::GetFPGATime();
    counter_ = 0;
}

void AutonomousCommand::Execute() noexcept
{
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    const uint deltaTime = (FPGATime - FPGATime_) / 1000; // ms

    if (deltaTime < 100) // 100ms
    {
        return;
    }

    FPGATime_ = FPGATime;

    // Will arrive here every 100ms, for 15s autonomous period.

    const units::pressure::pounds_per_square_inch_t pressure = m_infrastructure->GetPressure();

    if (!pressure_ && pressure < 75_psi)
    {
        return;
    }

    pressure_ = true;
    ++counter_;

    if (counter_ <= 5) // 500ms
    {
        if (counter_ == 5)
        {
            printf("Auto Stage 1.\n"); // XXX
        }

        // m_drive->Drive();

        m_feeder->LockIntake();
        m_feeder->RaiseIntake();

        m_feeder->Default(1.0);
        m_shooter->Default(0.5, 750.0);

        return;
    }

    if (counter_ <= 10) // 1s
    {
        if (counter_ == 10)
        {
            printf("Auto Stage 2.\n"); // XXX
        }

        m_feeder->DropIntake();
        m_feeder->LowerIntake();

        return;
    }

    if (counter_ <= 15) // 1.5s
    {
        if (counter_ == 15)
        {
            printf("Auto Stage 3.\n"); // XXX
        }

        m_feeder->LockIntake();

        return;
    }

    if (counter_ <= 20) // 2s
    {
        if (counter_ == 20)
        {
            printf("Auto Stage 4.\n"); // XXX
        }

        m_feeder->Default(1.0);

        return;
    }

    if (counter_ <= 30) // 3s
    {
        if (counter_ == 30)
        {
            printf("Auto Stage 5.\n"); // XXX
        }

        m_feeder->Fire();

        return;
    }

    if (counter_ <= 75) // 7.5s
    {
        if (counter_ == 75)
        {
            printf("Auto Stage 6.\n"); // XXX
        }

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
