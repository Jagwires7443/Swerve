// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/SwerveModule.h"

#include "Constants.h"

#include "AHRS.h"
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/SubsystemBase.h>
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

  DriveSubsystem(const DriveSubsystem &) = delete;
  DriveSubsystem &operator=(const DriveSubsystem &) = delete;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() noexcept override;

  void TestInit() noexcept;
  void TestExit() noexcept;
  void TestPeriodic() noexcept;

  // Obtain a non-owning pointer to the Command chooser, in the Test Mode UI.
  // Valid and unchanging, from return of TestInit() through destruction of
  // the associated DriveSubsystem().  This provides a way to inject Commands
  // into Test Mode, interactively.  If nullptr, TestInit() was never called.
  frc::SendableChooser<frc2::Command *> *TestModeChooser() noexcept { return m_chooser.get(); }

  // Obtain status of the overall swerve drive subsystem.
  bool GetStatus() const noexcept;

  // Test or simple autonomous (no motion planning) oriented methods;
  // note that these return false until the requested action has completed.
  void ResetDrive() noexcept;                                              // Zero drive distance
  void SetDriveBrakeMode(bool brake) noexcept;                             // Brake or coast
  bool ZeroModules() noexcept;                                             // Face forward
  bool SetTurnInPlace() noexcept;                                          // Orient modules for spin in-place
  bool SetLockWheelsX() noexcept;                                          // Orient modules for staying ("X")
  bool SetTurningPosition(const units::angle::degree_t position) noexcept; // Orient modules same direction
  bool SetTurnByAngle(units::degree_t angle) noexcept;                     // Spin, once set to spin in-place
  bool SetDriveDistance(units::length::meter_t distance) noexcept;         // Drive for specified distance

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

  // As above, but allow specification of a different center of rotation.
  void Drive(units::meters_per_second_t xSpeed,
             units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
             bool fieldRelative, units::meter_t x_center, units::meter_t y_center) noexcept;

  /**
   * Resets the drive encoder to zero, and the turning encoder based on the
   * absolute position sensor.
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
  units::degree_t GetHeading() noexcept;

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

  // Test mode method to power turning motors for characterization.
  void TestModeTurningVoltage(const double voltage) noexcept
  {
    m_testModeTurningVoltage = voltage;
    m_testModeDriveVoltage = 0.0;
  }

  // Test mode method to power drive motors for characterization.
  void TestModeDriveVoltage(const double voltage) noexcept
  {
    m_testModeTurningVoltage = 0.0;
    m_testModeDriveVoltage = voltage;
  }

  // Front: +x, Rear: -x; Left: +y, Right -y.  Zero heading is to the front
  // and +rotation is counter-clockwise.  This is all standard, although it
  // means the robot's front is along the x-axis, which is often pointed to
  // the right, as things are commonly drawn.  Rotate the page by 90 degrees.
  const frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d(+physical::kWheelBase / 2, +physical::kTrackWidth / 2),
      frc::Translation2d(+physical::kWheelBase / 2, -physical::kTrackWidth / 2),
      frc::Translation2d(-physical::kWheelBase / 2, +physical::kTrackWidth / 2),
      frc::Translation2d(-physical::kWheelBase / 2, -physical::kTrackWidth / 2)};

  // Need to derive from abstract Sendable class in order to be able to use the
  // PIDController UI element in Shuffleboard; note that this class doesn't
  // derive from frc::PIDController -- it's all down to inheritance and
  // properties.
  class TuningPID : public frc::Sendable, public frc::SendableHelper<TuningPID>
  {
  public:
    TuningPID(double p, double i, double d, double f) noexcept : m_p(p), m_i(i), m_d(d), m_f(f) {}

    TuningPID(const TuningPID &) = delete;
    TuningPID &operator=(const TuningPID &) = delete;

    void InitSendable(frc::SendableBuilder &builder)
    {
      builder.SetSmartDashboardType("PIDController");
      builder.AddDoubleProperty(
          "p", [&]() -> double { return m_p; }, [&](double value) -> void { m_p = value; });
      builder.AddDoubleProperty(
          "i", [&]() -> double { return m_i; }, [&](double value) -> void { m_i = value; });
      builder.AddDoubleProperty(
          "d", [&]() -> double { return m_d; }, [&](double value) -> void { m_d = value; });
      builder.AddDoubleProperty(
          "f", [&]() -> double { return m_f; }, [&](double value) -> void { m_f = value; });
      builder.AddDoubleProperty(
          "setpoint", [&]() -> double { return m_s; }, [&](double value) -> void { m_s = value; });
      builder.AddBooleanProperty(
          "enabled", [&]() -> bool { return m_e; }, [&](bool value) -> void { m_e = value; });
    }

    double GetP() const noexcept { return m_p; }
    double GetI() const noexcept { return m_i; }
    double GetD() const noexcept { return m_d; }
    double GetF() const noexcept { return m_f; }
    double GetS() const noexcept { return m_s; }
    bool GetE() const noexcept { return m_e; }

    void SetS(const double &value) noexcept { m_s = value; }
    void SetE(const bool value) noexcept { m_e = value; }

  private:
    double m_p{0.0};
    double m_i{0.0};
    double m_d{0.0};
    double m_f{0.0};
    double m_s{0.0};
    bool m_e{false};
  };

  // Need to derive from abstract Sendable class in order to be able to use the
  // Gyro UI element in Shuffleboard; note that this class does not derive from
  // frc::Gyro or frc::GyroBase -- it's all down to inheritance and properties.
  class HeadingGyro : public frc::Sendable, public frc::SendableHelper<HeadingGyro>
  {
  public:
    HeadingGyro() noexcept {}

    HeadingGyro(const HeadingGyro &) = delete;
    HeadingGyro &operator=(const HeadingGyro &) = delete;

    void InitSendable(frc::SendableBuilder &builder)
    {
      builder.SetSmartDashboardType("Gyro");
      builder.AddDoubleProperty(
          "Value", [&]() -> double { return m_value; }, nullptr);
    }

    void Set(const double &value) noexcept { m_value = value; }

  private:
    double m_value{0.0};
  };

private:
  void DoSafeIMU(const char *const what, std::function<void()> work) noexcept;

  // Test mode method to create graph tab for characterization.
  void CreateGraphTab(SwerveModule::GraphSelection graphSelection) noexcept;
  void UpdateGraphTab(SwerveModule::GraphSelection graphSelection) noexcept;

  // The gyro sensor.
  std::unique_ptr<AHRS> m_ahrs;

  // Four swerve modules.
  std::unique_ptr<SwerveModule> m_frontLeftSwerveModule;
  std::unique_ptr<SwerveModule> m_frontRightSwerveModule;
  std::unique_ptr<SwerveModule> m_rearLeftSwerveModule;
  std::unique_ptr<SwerveModule> m_rearRightSwerveModule;

  // Odometry class for tracking robot pose; 4 specifies the number of modules.
  std::unique_ptr<frc::SwerveDriveOdometry<4>> m_odometry;

  // Test Mode (only) last commanded states (for optional display).
  frc::SwerveModuleState m_commandedStateFrontLeft{};
  frc::SwerveModuleState m_commandedStateFrontRight{};
  frc::SwerveModuleState m_commandedStateRearLeft{};
  frc::SwerveModuleState m_commandedStateRearRight{};

  // Test Mode (only) instance of a "Gyro", needed for Shuffleboard UI.
  HeadingGyro m_frontLeftGyro;
  HeadingGyro m_frontRightGyro;
  HeadingGyro m_rearLeftGyro;
  HeadingGyro m_rearRightGyro;

  // Test Mode (only) instances of TuningPID, needed for Shuffleboard UI.
  std::unique_ptr<TuningPID> m_turningPositionPIDController;
  std::unique_ptr<TuningPID> m_drivePositionPIDController;
  std::unique_ptr<TuningPID> m_driveVelocityPIDController;

  double m_minProcessVariable{0.0};
  double m_maxProcessVariable{0.0};
  double m_minProcessError{0.0};
  double m_maxProcessError{0.0};
  double m_minProcessFirstDerivative{0.0};
  double m_maxProcessFirstDerivative{0.0};
  double m_minProcessSecondDerivative{0.0};
  double m_maxProcessSecondDerivative{0.0};

  // Test Mode (only) instance of test command chooser.
  std::unique_ptr<frc::SendableChooser<frc2::Command *>> m_chooser;
  frc2::Command *m_command{nullptr};

  // Last commanded drive inputs, for Test Mode display.
  double m_rotation{0.0};
  double m_x{0.0};
  double m_y{0.0};

  // Test Mode modification of behavior, allows low-level control and
  // modification of drive speed.
  bool m_run{true};
  double m_limit{1.0};
  bool m_graph{false};
  double m_testModeTurningVoltage{0.0};
  double m_testModeDriveVoltage{0.0};
  SwerveModule::GraphSelection m_graphSelection{SwerveModule::GraphSelection::kNone};

  // Test Mode (only) data, obtained but not owned.
  frc::ComplexWidget *m_frontLeftTurning{nullptr};
  frc::ComplexWidget *m_frontRightTurning{nullptr};
  frc::ComplexWidget *m_rearLeftTurning{nullptr};
  frc::ComplexWidget *m_rearRightTurning{nullptr};

  frc::ComplexWidget *m_turningPositionPID{nullptr};
  frc::ComplexWidget *m_drivePositionPID{nullptr};
  frc::ComplexWidget *m_driveVelocityPID{nullptr};

  frc::SimpleWidget *m_frontLeftDrive{nullptr};
  frc::SimpleWidget *m_frontRightDrive{nullptr};
  frc::SimpleWidget *m_rearLeftDrive{nullptr};
  frc::SimpleWidget *m_rearRightDrive{nullptr};

  frc::SimpleWidget *m_swerveRotation{nullptr};
  frc::SimpleWidget *m_swerveX{nullptr};
  frc::SimpleWidget *m_swerveY{nullptr};
  frc::SimpleWidget *m_swerveStatus{nullptr};
  frc::SimpleWidget *m_driveLimit{nullptr};
  frc::SimpleWidget *m_swerveEnable{nullptr};
  frc::SimpleWidget *m_displayMode{nullptr};
  frc::ComplexWidget *m_commandChooser{nullptr};

  frc::SimpleWidget *m_frontLeftGraph{nullptr};
  frc::SimpleWidget *m_frontRightGraph{nullptr};
  frc::SimpleWidget *m_rearLeftGraph{nullptr};
  frc::SimpleWidget *m_rearRightGraph{nullptr};

  std::string m_frontLeftGraphScroll;
  std::string m_frontRightGraphScroll;
  std::string m_rearLeftGraphScroll;
  std::string m_rearRightGraphScroll;
};
