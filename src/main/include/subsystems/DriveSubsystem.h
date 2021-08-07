// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/SwerveModule.h"

#include "Constants.h"

#include "AHRS.h"
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/array.h>

#include <memory>

class DriveSubsystem : public frc2::SubsystemBase
{
public:
  DriveSubsystem() noexcept;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() noexcept override;

  void TestInit() noexcept;
  void TestPeriodic() noexcept;

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction
   *                      (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to
   *                      the field.
   */
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative) noexcept;

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders() noexcept;

  /**
   * Sets the drive MotorControllers to a power from -1 to 1.
   *
   * Note that there is an essential ordering to everything to do with the four
   * swerve modules: Front Left, Front Right, Rear Left, and Rear Right.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> &desiredStates) noexcept;

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  units::degree_t GetHeading() const noexcept;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading() noexcept;

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate() noexcept;

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose() noexcept;

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose) noexcept;

  const frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d(physical::kWheelBase / 2, physical::kTrackWidth / 2),
      frc::Translation2d(physical::kWheelBase / 2, -physical::kTrackWidth / 2),
      frc::Translation2d(-physical::kWheelBase / 2, physical::kTrackWidth / 2),
      frc::Translation2d(-physical::kWheelBase / 2, -physical::kTrackWidth / 2)};

private:
  // The gyro sensor
  std::unique_ptr<AHRS> m_ahrs;

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  std::unique_ptr<SwerveModule> m_frontLeftSwerveModule;
  std::unique_ptr<SwerveModule> m_frontRightSwerveModule;
  std::unique_ptr<SwerveModule> m_rearLeftSwerveModule;
  std::unique_ptr<SwerveModule> m_rearRightSwerveModule;

  // Odometry class for tracking robot pose
  // 4 defines the number of modules
  std::unique_ptr<frc::SwerveDriveOdometry<4>> m_odometry;

  // Test Mode (only) instances of PIDController, needed for Shuffleboard UI.
  std::unique_ptr<frc2::PIDController> m_turningPositionPIDController;
  std::unique_ptr<frc2::PIDController> m_drivePositionPIDController;
  std::unique_ptr<frc2::PIDController> m_driveVelocityPIDController;

  // Test Mode (only) instances of network table directories for each PID
  // controller.  These provide direct access to some entries which are not
  // exposed via PIDController.
  std::shared_ptr<nt::NetworkTable> m_turningPositionPIDTable;
  std::shared_ptr<nt::NetworkTable> m_drivePositionPIDTable;
  std::shared_ptr<nt::NetworkTable> m_driveVelocityPIDTable;

  // Test Mode (only) data, obtained but not owned.
  frc::ComplexWidget *m_turningPositionPID = nullptr;
  frc::ComplexWidget *m_drivePositionPID = nullptr;
  frc::ComplexWidget *m_driveVelocityPID = nullptr;
};
