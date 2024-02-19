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

    m_Intake->StopIntake();

    m_shooter->StopShooter();

    finished_ = false;
    FPGATime_ = frc::RobotController::GetFPGATime();
    counter_ = 0;
}

void TimedAutoBase::End(bool interrupted) noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_Intake->StopIntake();

    m_shooter->StopShooter();
}

void TimedAutoBase::Execute() noexcept
{
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    const uint deltaTime = (FPGATime - FPGATime_) / 1000; // ms
}

frc2::CommandPtr TrajectoryAuto::TrajectoryAutoCommandFactory(DriveSubsystem *const drive, std::string_view name, frc::Trajectory &trajectory) noexcept
{
    frc2::SwerveControllerCommand<4> command{
        trajectory, [drive]() -> frc::Pose2d
        { return drive->GetPose(); },
        drive->kDriveKinematics,
        frc::PIDController{0.6, 0, 0},
        frc::PIDController{0.6, 0, 0},
        frc::ProfiledPIDController<units::radians>{pidf::kDriveThetaP, pidf::kDriveThetaI, pidf::kDriveThetaD, frc::TrapezoidProfile<units::angle::radians>::Constraints{pidf::kDriveThetaMaxVelocity, pidf::kDriveThetaMaxAcceleration}},
        // [drive]() -> frc::Rotation2d "desiredRotation"
        [drive](std::array<frc::SwerveModuleState, 4> states) -> void
        { drive->SetModuleStates(states); },
        {drive}};

    command.SetName(name);

    return std::move(command).ToPtr();
}
