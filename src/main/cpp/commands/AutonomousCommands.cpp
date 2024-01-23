// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <units/angle.h>

#if 0
frc2::CommandPtr TrajectoryAuto::TrajectoryAutoCommandFactory(DriveSubsystem *const drive, std::string_view name, frc::Trajectory &trajectory) noexcept
{
    frc2::SwerveControllerCommand<4> command
    {
trajectory, [drive]() -> frc::Pose2d
        { return drive->GetPose(); },
        drive->kDriveKinematics,
        frc2::PIDController{0.6, 0, 0},
        frc2::PIDController{0.6, 0, 0},
        frc::ProfiledPIDController<units::radians>{pidf::kDriveThetaP, pidf::kDriveThetaI, pidf::kDriveThetaD, frc::TrapezoidProfile<units::angle::radians>::Constraints{pidf::kDriveThetaMaxVelocity, pidf::kDriveThetaMaxAcceleration}},
        // [drive]() -> frc::Rotation2d "desiredRotation"
        [drive](std::array<frc::SwerveModuleState, 4> states) -> void
        { drive->SetModuleStates(states); },
        {drive}
    };

    command.SetName(name);

    return std::move(command).ToPtr();
}
#endif

