// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <units/angle.h>

void TimedAutoBase::Initialize() noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
    m_drive->ResetDrive();
    m_drive->ZeroHeading();

    m_feeder->Default(0.0);
    m_feeder->LockIntake();
    m_feeder->RaiseIntake();

    m_shooter->Stop();

    m_infrastructure->SetLEDPattern(95);

    pressure_ = false;
    finished_ = false;
    FPGATime_ = frc::RobotController::GetFPGATime();
    counter_ = 0;
}

void TimedAutoBase::End(bool interrupted) noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_feeder->Default(0.0);

    m_shooter->Stop();

    m_infrastructure->SetLEDPattern(95);
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
        m_infrastructure->SetLEDPattern(78);

        return;
    }

    // Assuming pneumatics were prefilled, counter runs 1 - ~150.
    if (Iteration(++counter_))
    {
        m_infrastructure->SetLEDPattern(95);

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

        m_infrastructure->SetLEDPattern(79);

        return false;
    }

    // Drop intake (first ball is preloaded).
    if (counter <= 10) // 0.5 - 1.0s
    {
        m_feeder->DropIntake();

        m_infrastructure->SetLEDPattern(80);

        return false;
    }

    // Reextend drop catches, lower intake.
    if (counter <= 15) // 1.0 - 1.5s
    {
        m_feeder->LockIntake();
        m_feeder->LowerIntake();

        m_infrastructure->SetLEDPattern(81);

        return false;
    }

    // Back up and spin up shooter.
    if (counter <= 35) // 1.5 - 3.5s
    {
        m_drive->Drive(-0.55_mps, 0_mps, 0_deg_per_s, false);

        m_feeder->Default(0.0);

        m_shooter->Default(1.0, 1125.0);

        m_infrastructure->SetLEDPattern(82);

        return false;
    }

    // Stop.
    if (counter <= 40) // 3.5 - 4.0s
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

        m_infrastructure->SetLEDPattern(83);

        return false;
    }

    // Take the shot!
    if (counter <= 60) // 4.0 - 6.0s
    {
        m_feeder->Fire();

        m_infrastructure->SetLEDPattern(84);

        return false;
    }

    return true;
}

bool TwoBallAuto::Iteration(const uint counter) noexcept
{
    // Sit still.
    if (counter <= 5) // 0.0 - 0.5s
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

        m_feeder->Default(0.0);
        m_feeder->LockIntake();
        m_feeder->RaiseIntake();

        m_shooter->Stop();

        m_infrastructure->SetLEDPattern(79);

        return false;
    }

    // Drop intake (first ball is preloaded).
    if (counter <= 10) // 0.5 - 1.0s
    {
        m_feeder->DropIntake();

        m_infrastructure->SetLEDPattern(80);

        return false;
    }

    // Reextend drop catches, lower intake.
    if (counter <= 15) // 1.0 - 1.5s
    {
        m_feeder->LockIntake();
        m_feeder->LowerIntake();

        m_infrastructure->SetLEDPattern(81);

        return false;
    }

    // Drive forward, run intake.
    if (counter <= 30) // 1.5 - 3.0s
    {
        m_drive->Drive(0.55_mps, 0_mps, 0_deg_per_s, false);

        m_feeder->Default(1.0);

        m_infrastructure->SetLEDPattern(82);

        return false;
    }

    // Stop.
    if (counter <= 35) // 3.0 - 3.5s
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

        m_infrastructure->SetLEDPattern(83);

        return false;
    }

    // Turn around and spin up shooter.
    if (counter < 85) // 3.5 - 8.5s
    {
        if (m_drive->SetTurnToAngle(180_deg))
        {
            m_infrastructure->SetLEDPattern(85);
        }
        else
        {
            m_infrastructure->SetLEDPattern(84);
        }

        m_shooter->Default(1.0, 1125.0);

        return false;
    }
    else if (counter == 85)
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        m_drive->ResetDrive();
        m_drive->ZeroHeading();

        m_infrastructure->SetLEDPattern(86);

        m_shooter->Default(1.0, 1125.0);

        return false;
    }

    // Spin up shooter.
    if (counter <= 90) // 8.5 - 9.0s
    {
        m_shooter->Default(1.0, 1125.0);

        m_feeder->Default(0.0);

        m_infrastructure->SetLEDPattern(87);

        return false;
    }

    // Take first shot!
    if (counter <= 110) // 9.0 - 11.0s
    {
        m_feeder->Fire();

        m_infrastructure->SetLEDPattern(88);

        return false;
    }

    // Feed and hold second shot.
    if (counter <= 120) // 11.0 - 12.0s
    {
        m_feeder->Default(1.0);

        m_infrastructure->SetLEDPattern(89);

        return false;
    }

    // Take second, buzzer beater shot!
    if (counter <= 150) // 13.0 - 15.0s
    {
        m_feeder->Fire();

        m_infrastructure->SetLEDPattern(90);

        return false;
    }

    return true;
}

frc2::CommandPtr TrajectoryAuto::TrajectoryAutoCommandFactory(DriveSubsystem *const drive, std::string_view name, frc::Trajectory &trajectory) noexcept
{
    frc2::SwerveControllerCommand<4> command{
        trajectory, [drive]() -> frc::Pose2d
        { return drive->GetPose(); },
        drive->kDriveKinematics,
        frc2::PIDController{0.6, 0, 0},
        frc2::PIDController{0.6, 0, 0},
        frc::ProfiledPIDController<units::radians>{pidf::kDriveThetaP, pidf::kDriveThetaI, pidf::kDriveThetaD, frc::TrapezoidProfile<units::angle::radians>::Constraints{pidf::kDriveThetaMaxVelocity, pidf::kDriveThetaMaxAcceleration}},
        // [drive]() -> frc::Rotation2d "desiredRotation"
        [drive](std::array<frc::SwerveModuleState, 4> states) -> void
        { drive->SetModuleStates(states); },
        {drive}};

    command.SetName(name);

    return std::move(command).ToPtr();
}
