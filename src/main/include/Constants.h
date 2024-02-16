#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace physical
{
    // Alignment constants, for each swerve module.  Specified on [-180, 180)
    // "count" scale, in deg.  From driver station view: +clockwise -counterclockwise.
    constexpr units::degree_t kFrontLeftAlignmentOffset = -92_deg;
    constexpr units::degree_t kFrontRightAlignmentOffset = 93_deg;
    constexpr units::degree_t kRearLeftAlignmentOffset = 26_deg;
    constexpr units::degree_t kRearRightAlignmentOffset = 131_deg;

    // SDS Mk3 Standard (or Fast) Gear Ratio: 8.16:1 (or 6.86:1);
    // Nominal Wheel Diameter (4"): =0.1016m;
    // Nominal Wheel Circumference (pi * Diameter): ~0.3192m;
    // 8.16 / 0.3192 => ~25.57.

    // This should be empirically determined!  This is just an initial guess.
    // This is used for both distance and velocity control.  If this is off, it
    // will throw off kMaxDriveSpeed and kMaxTurnRate, as well as drive values.
    constexpr units::meter_t kDriveMetersPerRotation = 1.0_m / 25.57;

    // SDS Mk3 Standard (or Fast) Max Free Speed: 12.1 (or 14.4) feet/second;
    // This is an upper bound, for various reasons.  It needs to be empirically
    // measured.  Half of theoretical free speed is a reasonable starting value
    // (since something in the ballpark is needed here in order to to drive).
    constexpr units::meters_per_second_t kMaxDriveSpeed = 12.1_fps / 1.25;

    // For a square drive base, with +/-11.25" x/y coordinates for each of four
    // swerve modules, the radius of the circle going through all modules is:
    // sqrt((11.25")^2 + (11.25")^2) ~= 15.91"; the circumference of such a
    // circle is 2*pi*15.91" ~= 99.96".

    // This is used for rotating the robot in place, about it's center.  This
    // may need to be empirically adjusted, but check kDriveMetersPerRotation
    // before making any adjustment here.
    constexpr units::meter_t kDriveMetersPerTurningCircle = 99.96_in;

    // This is the maximum rotational speed -- not of a swerve module, but of
    // the entire robot.  This is a function of the maximum drive speed and the
    // geometry of the robot.  This will occur when the robot spins in place,
    // around the center of a circle which passes through all the drive modules
    // (if there is no single such circle, things are analogous).  If the drive
    // modules are turned to be tangential to this circle and run at maximum,
    // the robot is rotating as fast as possible.  This can be derived from
    // kMaxDriveSpeed and the geometry and does not have to be directly
    // measured.  It is a good idea to check this value empirically though.

    // So the maximum rotational velocity (spinning in place) is kMaxDriveSpeed
    // / kDriveMetersPerTurningCircle * 360 degrees.  This should not need to
    // be empirically adjusted (but check).
    constexpr units::degrees_per_second_t kMaxTurnRate =
        kMaxDriveSpeed / kDriveMetersPerTurningCircle * 360.0_deg;

    // Drivebase geometry: distance between centers of right and left wheels on
    // robot; distance between centers of front and back wheels on robot.
    constexpr units::meter_t kTrackWidth = 24.5_in;
    constexpr units::meter_t kWheelBase = 24.5_in;

    // CAN ID and Digital I/O Port assignments.
    constexpr int kFrontRightDriveMotorCanID = 1;
    constexpr int kFrontRightTurningMotorCanID = 2;
    constexpr int kRearRightDriveMotorCanID = 3;
    constexpr int kRearRightTurningMotorCanID = 4;
    constexpr int kRearLeftDriveMotorCanID = 5;
    constexpr int kRearLeftTurningMotorCanID = 6;
    constexpr int kFrontLeftDriveMotorCanID = 7;
    constexpr int kFrontLeftTurningMotorCanID = 8;
    constexpr int kFrontRightTurningEncoderPort = 10;
    constexpr int kRearRightTurningEncoderPort = 11;
    constexpr int kRearLeftTurningEncoderPort = 12;
    constexpr int kFrontLeftTurningEncoderPort = 13;
    

    

    // These can flip because of gearing.
    constexpr bool kDriveMotorInverted = false;
    constexpr bool kTurningMotorInverted = false;
}

namespace firmware
{
    constexpr int kSparkFirmwareVersion = 0x01060003; // Version 1.6.3.
}

namespace pidf
{
    constexpr units::degrees_per_second_t kTurningPositionMaxVelocity = 2750.0_deg_per_s;
    constexpr units::degrees_per_second_squared_t kTurningPositionMaxAcceleration = 20000.0_deg_per_s_sq;
    constexpr double kTurningPositionP = 0.00395; // originally 0.006 concrete 0.00395
    constexpr double kTurningPositionF = 0.0; // originally 0.003 concrete 0.003
    constexpr double kTurningPositionI = 0.0;
    constexpr double kTurningPositionIZ = 0.0;
    constexpr double kTurningPositionIM = 0.0;
    constexpr double kTurningPositionD = 0.000;
    constexpr double kTurningPositionDF = 0.0;

    constexpr double kDrivePositionMaxVelocity = 5700.0;     // Rotations per minute.
    constexpr double kDrivePositionMaxAcceleration = 1000.0; // Rotations per minute per second.
    constexpr double kDrivePositionP = 0.000;
    constexpr double kDrivePositionF = 0.0;
    constexpr double kDrivePositionI = 0.0;
    constexpr double kDrivePositionIZ = 0.0;
    constexpr double kDrivePositionIM = 0.0;
    constexpr double kDrivePositionD = 0.0;
    constexpr double kDrivePositionDF = 0.0;

    constexpr double kDriveVelocityMaxVelocity = 5700.0;
    constexpr double kDriveVelocityMaxAcceleration = 1000.0;
    constexpr double kDriveVelocityMaxJerk = 1.0;
    constexpr double kDriveVelocityP = 0.0;
    constexpr double kDriveVelocityF = 0.0;
    constexpr double kDriveVelocityI = 0.0;
    constexpr double kDriveVelocityIZ = 0.0;
    constexpr double kDriveVelocityIM = 0.0;
    constexpr double kDriveVelocityD = 0.0;
    constexpr double kDriveVelocityDF = 0.0;

    constexpr units::degrees_per_second_t kDriveThetaMaxVelocity = 45.0_deg_per_s;
    constexpr units::degrees_per_second_squared_t kDriveThetaMaxAcceleration = 450.0_deg_per_s_sq;
    constexpr double kDriveThetaP = 0.10;
    constexpr double kDriveThetaF = 0.005;
    constexpr double kDriveThetaI = 0.0;
    constexpr double kDriveThetaD = 0.0;
}

namespace nonDrive
{

}

namespace shooter
{
    constexpr int kLeftShooterMotorCanID = 14;
    constexpr int kRightShooterMotorCanID = 15;

    constexpr int kShooterMotorSpeed = 0;

}

namespace intake
{
    constexpr int kIntakeArmMotorCanID = 16;
    constexpr int kIntakeSpinMotorCanID = 17;

    /* NOTE!!!: The intake arm values are NOT final, are subject to change
    after testing*/
    constexpr units::degree_t kIntakeArmHome = 90.0_deg;
    constexpr units::degree_t kIntakeArmPickup = 180.0_deg;
    constexpr units::degree_t kIntakeArmLoad = 0.0_deg;
}

