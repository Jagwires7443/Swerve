// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"

void TimedAutoBase::Initialize() noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_feeder->Default(0.0);
    m_feeder->LockIntake();
    m_feeder->RaiseIntake();

    m_shooter->Stop();

    FPGATime_ = frc::RobotController::GetFPGATime();
    counter_ = 0;
}

void TimedAutoBase::End(bool interrupted) noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_feeder->Default(0.0);

    m_shooter->Stop();
}

void TimedAutoBase::Execute() noexcept
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

    if (pressure >= 75_psi)
    {
        pressure_ = true;
    }

    if (!pressure_)
    {
        return;
    }

    // Assuming pneumatics were prefilled, counter runs 1 - ~150.
    const bool finished = Iteration(++counter_);

    if (finished)
    {
        finished_ = true;
    }
}

bool OneBallAuto::Iteration(const uint counter) noexcept
{
    // Sit still and run intake/elevator.
    if (counter <= 5) // 0.0 - 0.5s
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

        m_feeder->Default(1.0);
        m_feeder->LockIntake();
        m_feeder->RaiseIntake();

        m_shooter->Stop();

        return false;
    }

    // Drop intake (first ball is preloaded).
    if (counter <= 10) // 0.5 - 1.0s
    {
        m_feeder->DropIntake();

        return false;
    }

    // Reextend drop catches, lower intake.
    if (counter <= 15) // 1.0 - 1.5s
    {
        m_feeder->LockIntake();
        m_feeder->LowerIntake();

        return false;
    }

    // Back up and spin up shooter.
    if (counter <= 35) // 1.5 - 3.5s
    {
        m_drive->Drive(-0.55_mps, 0_mps, 0_deg_per_s, false);

        m_feeder->Default(0.0);

        m_shooter->Default(1.0, 930.0);

        return false;
    }

    // Stop.
    if (counter <= 40) // 3.5 - 4.0s
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

        return false;
    }

    // Take first shot!
    if (counter <= 60) // 4.0 - 6.0s
    {
        m_feeder->Fire();

        return false;
    }

    return true;
}

bool TwoBallAuto::Iteration(const uint counter) noexcept
{
    // Turn around.
    if (counter <= 40) // 0.0 - 4.0s
    {
        (void)m_drive->SetTurnToAngle(90_deg);

        return false;
    }

    return true;
}
