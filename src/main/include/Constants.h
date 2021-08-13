// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
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
    // Alignment constants, for each swerve module.  Specified on [0, 4095]
    // "count" scale, in (dimensionless) angular units.
    constexpr int kFrontLeftAlignmentOffset = 3029;  // 981
    constexpr int kFrontRightAlignmentOffset = 1012; // 3060
    constexpr int kRearLeftAlignmentOffset = 2862;   // 814
    constexpr int kRearRightAlignmentOffset = 3515;  // 1467

    // XXX explain these -- what they do, effects of too high or too low, empirically set

    // SDS Mk3 Standard (or Fast) Gear Ratio: 8.16:1 (or 6.86:1);
    // Nominal Wheel Diameter (4"): =0.1016m;
    // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
    // 8.16 / 0.3192 => ~25.57.

    // This should be empirically determined!  This is just an initial guess.
    // This is used for both distance and velocity control.  If this is off, it
    // will throw off kMaxDriveSpeed and kMaxTurnRate, as well as drive values.
    constexpr units::meter_t kDriveMetersPerRotation = 1_m / 25.57;

    // SDS Mk3 Standard (or Fast) Max Free Speed: 12.1 (or 14.4) feet/second;
    // This is an upper bound, for various reasons.  It needs to be empirically
    // measured.  Half of theoretical free speed is a reasonable starting value
    // (since something in the ballpark is needed here in order to to drive).
    constexpr units::meters_per_second_t kMaxDriveSpeed = 12.1_fps / 2;

    // This is the maximum rotational speed -- not of a swerve module, but of
    // the entire robot.  This is a function of the maximum drive speed and the
    // geometry of the robot.  This will occur when the robot spins in place,
    // around the center of a circle which passes through all the drive modules
    // (if there is no single such circle, things are analogous).  If the drive
    // modules are turned to be tangential to this circle and run at maximum,
    // the robot is rotating as fast as possible.  This can be derived from
    // kMaxDriveSpeed and the geometry and does not have to be directly
    // measured.  It is a good idea to check this value empirically though.

    // For a square drive base, with +/-11.25" x/y coordinates for each of four
    // swerve modules, the radius of the circle going through all modules is:
    // sqrt((11.25")^2 + (11.25")^2) ~= 15.91"; the circumference of such a
    // circle is 2*pi*15.91" ~= 99.96".
    // So, the maximum rotational velocity is kMaxDriveSpeed / 99.96" * 360
    // degrees.
    constexpr units::degrees_per_second_t kMaxTurnRate = kMaxDriveSpeed / 99.96_in * 360_deg;

    // Drivebase geometry: distance between centers of right and left wheels on
    // robot; distance between centers of front and back wheels on robot.
    constexpr units::meter_t kTrackWidth = 22.5_in;
    constexpr units::meter_t kWheelBase = 22.5_in;

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
    constexpr double kTurningPositionP = 0.001;
    constexpr double kTurningPositionI = 0.0;
    constexpr double kTurningPositionIZ = 0.0;
    constexpr double kTurningPositionIM = 0.0;
    constexpr double kTurningPositionD = 0.0;
    constexpr double kTurningPositionDF = 0.0;
    constexpr double kTurningPositionF = 0.0;

    constexpr double kDrivePositionP = 0.0;
    constexpr double kDrivePositionI = 0.0;
    constexpr double kDrivePositionIZ = 0.0;
    constexpr double kDrivePositionIM = 0.0;
    constexpr double kDrivePositionD = 0.0;
    constexpr double kDrivePositionDF = 0.0;
    constexpr double kDrivePositionF = 0.0;

    constexpr double kDriveVelocityP = 0.0;
    constexpr double kDriveVelocityI = 0.0;
    constexpr double kDriveVelocityIZ = 0.0;
    constexpr double kDriveVelocityIM = 0.0;
    constexpr double kDriveVelocityD = 0.0;
    constexpr double kDriveVelocityDF = 0.0;
    constexpr double kDriveVelocityF = 0.0;
}
