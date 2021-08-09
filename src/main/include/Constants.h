// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/length.h>
#include <units/velocity.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */
namespace physical
{
    // Alignment constants, for each swerve module.
    constexpr int kFrontLeftAlignmentOffset = 981;
    constexpr int kFrontRightAlignmentOffset = 3060;
    constexpr int kRearLeftAlignmentOffset = 814;
    constexpr int kRearRightAlignmentOffset = 1467;

    // SDS Mk3 Standard (or Fast) Max Free Speed: 12.1 (or 14.4) feet/second;
    // This is an upper bound, for various reasons.
    constexpr units::meters_per_second_t kMaxDriveSpeed = 3.688_mps;

    // SPARK MAX counts/revolution: 42;
    // SDS Mk3 Standard (or Fast) Gear Ratio: 8.16:1 (or 6.86:1);
    // Nominal Wheel Diameter (4"): =0.1016m;
    // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
    // 42 * 8.16 / 0.3192 => ~1074.
    // This should be empirically determined!
    constexpr double kDriveCountsPerMeter = 1074.0;

    // Drivebase geometry: distance between centers of right and left wheels on
    // robot; distance between centers of front and back wheels on robot.
    constexpr units::meter_t kTrackWidth = 0.5715_m; // 22.5"
    constexpr units::meter_t kWheelBase = 0.5715_m;  // 22.5"

    // CAN ID and Digital I/O Port assignments.
    constexpr int kFrontLeftDriveMotorCanID = 7;
    constexpr int kFrontLeftTurningMotorCanID = 8;
    constexpr int kFrontLeftTurningEncoderPort = 0;
    constexpr int kFrontRightDriveMotorCanID = 1;
    constexpr int kFrontRightTurningMotorCanID = 2;
    constexpr int kFrontRightTurningEncoderPort = 1;
    constexpr int kRearLeftDriveMotorCanID = 5;
    constexpr int kRearLeftTurningMotorCanID = 6;
    constexpr int kRearLeftTurningEncoderPort = 2;
    constexpr int kRearRightDriveMotorCanID = 3;
    constexpr int kRearRightTurningMotorCanID = 4;
    constexpr int kRearRightTurningEncoderPort = 3;

    // These can flip because of gearing or wiring.
    constexpr bool kDriveMotorInverted = false;
    constexpr bool kTurningMotorInverted = false;
    constexpr bool kTurningEncoderInverted = true;
}

namespace firmware
{
    constexpr int kSparkMaxFirmwareVersion = 0x1050002;
}

namespace pidf
{
    constexpr double kTurningPositionP = 0.0;
    constexpr double kTurningPositionI = 0.0;
    constexpr double kTurningPositionD = 0.0;
    constexpr double kTurningPositionF = 0.0;

    constexpr double kDrivePositionP = 0.0;
    constexpr double kDrivePositionI = 0.0;
    constexpr double kDrivePositionD = 0.0;
    constexpr double kDrivePositionF = 0.0;

    constexpr double kDriveVelocityP = 0.0;
    constexpr double kDriveVelocityI = 0.0;
    constexpr double kDriveVelocityD = 0.0;
    constexpr double kDriveVelocityF = 0.0;
}
