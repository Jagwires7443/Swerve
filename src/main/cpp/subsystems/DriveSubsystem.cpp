// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// See https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html.
// See https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/swerve-drive-odometry.html.
// See https://github.com/wpilibsuite/allwpilib/blob/main/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/subsystems/DriveSubsystem.cpp.

#include "subsystems/DriveSubsystem.h"

#include "subsystems/SwerveModule.h"

#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>

#include <chrono>
#include <thread>

DriveSubsystem::DriveSubsystem() noexcept
{
  // Set up the "navX" IMU first, so there's more time before it is used later.
  // See https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/.
  // Allow up to 20 seconds for callibration; it is supposed to be much faster,
  // when the IMU is kept still.  Consider using lights or other feedback so it
  // is very clear when this is occurring.
#if 0
  try
  {
    m_ahrs = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

    using namespace std::chrono_literals;

    const std::chrono::steady_clock::time_point wait_until = std::chrono::steady_clock::now() + 20s;
    while (!m_ahrs->IsConnected() || m_ahrs->IsCalibrating())
    {
      std::this_thread::sleep_for(100ms);
      // XXX break if here too long (by throwing -- or want best effort if it's there?)
      if (std::chrono::steady_clock::now() >= wait_until)
      {
        break;
      }
    }
  }
  catch (...)
  {
    m_ahrs = nullptr;
  }

  // Initial position (third parameter) defaulted to "frc::Pose2d()"; initial
  // angle (second parameter) is automatically zeroed by navX initialization.
  m_odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(kDriveKinematics, 0_deg);
#endif

  // These are last, so there can be no movement from the swerve modules.
  m_frontLeftSwerveModule = std::make_unique<SwerveModule>(
      "Front Left",
      physical::kFrontLeftDriveMotorCanID,
      physical::kFrontLeftTurningMotorCanID,
      physical::kFrontLeftTurningEncoderPort,
      physical::kFrontLeftAlignmentOffset);

  m_frontRightSwerveModule = std::make_unique<SwerveModule>(
      "Front Right",
      physical::kFrontRightDriveMotorCanID,
      physical::kFrontRightTurningMotorCanID,
      physical::kFrontRightTurningEncoderPort,
      physical::kFrontRightAlignmentOffset);

  m_rearLeftSwerveModule = std::make_unique<SwerveModule>(
      "Rear Left",
      physical::kRearLeftDriveMotorCanID,
      physical::kRearLeftTurningMotorCanID,
      physical::kRearLeftTurningEncoderPort,
      physical::kRearLeftAlignmentOffset);

  m_rearRightSwerveModule = std::make_unique<SwerveModule>(
      "Rear Right",
      physical::kRearRightDriveMotorCanID,
      physical::kRearRightTurningMotorCanID,
      physical::kRearRightTurningEncoderPort,
      physical::kRearRightAlignmentOffset);
}

void DriveSubsystem::Periodic() noexcept
{
#if 0
  // Implementation of subsystem periodic method goes here.
  m_odometry->Update(m_ahrs->GetRotation2d(), m_frontLeftSwerveModule->GetState(),
                     m_frontRightSwerveModule->GetState(), m_rearLeftSwerveModule->GetState(),
                     m_rearRightSwerveModule->GetState());
#endif
}

frc::Pose2d DriveSubsystem::GetPose() noexcept { return m_odometry->GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) noexcept
{
  m_odometry->ResetPosition(pose, m_ahrs->GetRotation2d());
}

void DriveSubsystem::TestInit() noexcept
{
  m_frontLeftSwerveModule->TestInit();
  m_frontRightSwerveModule->TestInit();
  m_rearLeftSwerveModule->TestInit();
  m_rearRightSwerveModule->TestInit();

  std::printf("Drive Subsystem Test Mode Init... ");

  m_turningPositionPIDController = std::make_unique<TuningPID>(pidf::kTurningPositionP,
                                                               pidf::kTurningPositionI,
                                                               pidf::kTurningPositionD,
                                                               pidf::kTurningPositionF);

  m_drivePositionPIDController = std::make_unique<TuningPID>(pidf::kDrivePositionP,
                                                             pidf::kDrivePositionI,
                                                             pidf::kDrivePositionD,
                                                             pidf::kDrivePositionF);

  m_driveVelocityPIDController = std::make_unique<TuningPID>(pidf::kDriveVelocityP,
                                                             pidf::kDriveVelocityI,
                                                             pidf::kDriveVelocityD,
                                                             pidf::kDrivePositionF);

  // XXX Enough test patterns to tune PID, plus maybe one or two fancy ones
  frc::ShuffleboardTab &shuffleboardTab = frc::Shuffleboard::GetTab("Swerve");

  frc::ShuffleboardLayout &shuffleboardLayoutSwerveTurning =
      shuffleboardTab.GetLayout("Swerve Turning",
                                frc::BuiltInLayouts::kGrid)
          .WithPosition(0, 0)
          .WithSize(9, 13)
          .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(2.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutPIDSettings =
      shuffleboardTab.GetLayout("PID Settings",
                                frc::BuiltInLayouts::kGrid)
          .WithPosition(9, 0)
          .WithSize(19, 7)
          .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(1.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutSwerveDrive =
      shuffleboardTab.GetLayout("Swerve Drive",
                                frc::BuiltInLayouts::kGrid)
          .WithPosition(9, 7)
          .WithSize(7, 6)
          .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(2.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutControls =
      shuffleboardTab.GetLayout("Controls",
                                frc::BuiltInLayouts::kGrid)
          .WithPosition(16, 7)
          .WithSize(12, 6)
          .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(4.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

  m_frontLeftTurning = &shuffleboardLayoutSwerveTurning.Add("Front Left", m_frontLeftGyro)
                            .WithPosition(0, 0)
                            .WithWidget(frc::BuiltInWidgets::kGyro);

  m_frontRightTurning = &shuffleboardLayoutSwerveTurning.Add("Front Right", m_frontRightGyro)
                             .WithPosition(0, 1)
                             .WithWidget(frc::BuiltInWidgets::kGyro);

  m_rearLeftTurning = &shuffleboardLayoutSwerveTurning.Add("Rear Left", m_rearLeftGyro)
                           .WithPosition(0, 1)
                           .WithWidget(frc::BuiltInWidgets::kGyro);

  m_rearRightTurning = &shuffleboardLayoutSwerveTurning.Add("Rear Right", m_rearRightGyro)
                            .WithPosition(1, 1)
                            .WithWidget(frc::BuiltInWidgets::kGyro);

  m_turningPositionPID = &shuffleboardLayoutPIDSettings.Add("Turning Position", *m_turningPositionPIDController)
                              .WithPosition(0, 0)
                              .WithWidget(frc::BuiltInWidgets::kPIDController);

  m_drivePositionPID = &shuffleboardLayoutPIDSettings.Add("Drive Distance", *m_drivePositionPIDController)
                            .WithPosition(1, 0)
                            .WithWidget(frc::BuiltInWidgets::kPIDController);

  m_driveVelocityPID = &shuffleboardLayoutPIDSettings.Add("Drive Velocity", *m_driveVelocityPIDController)
                            .WithPosition(2, 0)
                            .WithWidget(frc::BuiltInWidgets::kPIDController);

  m_frontLeftDrive = &shuffleboardLayoutSwerveDrive.Add("Front Left", 0.0)
                          .WithPosition(0, 0)
                          .WithWidget(frc::BuiltInWidgets::kNumberBar);

  m_frontRightDrive = &shuffleboardLayoutSwerveDrive.Add("Front Right", 0.0)
                           .WithPosition(1, 0)
                           .WithWidget(frc::BuiltInWidgets::kNumberBar);

  m_rearLeftDrive = &shuffleboardLayoutSwerveDrive.Add("Rear Left", 0.0)
                         .WithPosition(0, 1)
                         .WithWidget(frc::BuiltInWidgets::kNumberBar);

  m_rearRightDrive = &shuffleboardLayoutSwerveDrive.Add("Rear Right", 0.0)
                          .WithPosition(1, 1)
                          .WithWidget(frc::BuiltInWidgets::kNumberBar);

  // rotation, X, limit, distance/velocity
  // status, Y, chooser, run/stop

  m_turningPositionPIDTable = nt::NetworkTableInstance::GetDefault().GetTable(
      "Shuffleboard/Swerve/PID Settings/Turning Position");
  if (!m_turningPositionPIDTable)
  {
    std::printf(" Turning Position Table error...");
  }

  m_drivePositionPIDTable = nt::NetworkTableInstance::GetDefault().GetTable(
      "Shuffleboard/Swerve/PID Settings/Drive Distance");
  if (!m_drivePositionPIDTable)
  {
    std::printf(" Drive Distance Table error...");
  }

  m_driveVelocityPIDTable = nt::NetworkTableInstance::GetDefault().GetTable(
      "Shuffleboard/Swerve/PID Settings/Drive Velocity");
  if (!m_driveVelocityPIDTable)
  {
    std::printf(" Drive Velocity Table error...");
  }

  std::printf("OK.\n");
}

void DriveSubsystem::TestPeriodic() noexcept
{
  m_frontLeftSwerveModule->TestPeriodic();
  m_frontRightSwerveModule->TestPeriodic();
  m_rearLeftSwerveModule->TestPeriodic();
  m_rearRightSwerveModule->TestPeriodic();

  m_frontLeftGyro.Set(m_frontLeftSwerveModule->GetTurningPosition().to<double>());
  m_frontRightGyro.Set(m_frontRightSwerveModule->GetTurningPosition().to<double>());
  m_rearLeftGyro.Set(m_rearLeftSwerveModule->GetTurningPosition().to<double>());
  m_rearRightGyro.Set(m_rearRightSwerveModule->GetTurningPosition().to<double>());

  m_frontLeftDrive->GetEntry().SetDouble(m_frontLeftSwerveModule->GetDriveVelocity().to<double>());
  m_frontRightDrive->GetEntry().SetDouble(m_frontRightSwerveModule->GetDriveVelocity().to<double>());
  m_rearLeftDrive->GetEntry().SetDouble(m_rearLeftSwerveModule->GetDriveVelocity().to<double>());
  m_rearRightDrive->GetEntry().SetDouble(m_rearRightSwerveModule->GetDriveVelocity().to<double>());

  if (m_turningPositionPIDTable)
  {
    // XXX refactor so there's less work here (move to init)?
    nt::NetworkTableEntry fEntry = m_turningPositionPIDTable->GetEntry("f");
    nt::NetworkTableEntry enabledEntry = m_turningPositionPIDTable->GetEntry("enabled");
    std::shared_ptr<nt::Value> fValue = fEntry.GetValue();
    std::shared_ptr<nt::Value> enabledValue = enabledEntry.GetValue();

    // if (enabledValue && enabledValue->IsBoolean() && enabledValue->GetBoolean()) {}

    // if (fValue && fValue->IsDouble()) {}
  }

  if (m_turningPositionPIDController->GetE())
  {
    double p = m_turningPositionPIDController->GetP();
    double i = m_turningPositionPIDController->GetI();
    double d = m_turningPositionPIDController->GetD();
    double f = m_turningPositionPIDController->GetF();
    double s = m_turningPositionPIDController->GetS();

    m_turningPositionPIDController->SetE(false);

    std::printf("**** PID: %f/%f/%f/%f/%f\n", p, i, d, f, s);

    // XXX fill in constants for all "0.0"
    m_frontLeftSwerveModule->TurningPositionPID(p, i, 0.0, 0.0, d, 0.0, f);
    m_frontRightSwerveModule->TurningPositionPID(p, i, 0.0, 0.0, d, 0.0, f);
    m_rearLeftSwerveModule->TurningPositionPID(p, i, 0.0, 0.0, d, 0.0, f);
    m_rearRightSwerveModule->TurningPositionPID(p, i, 0.0, 0.0, d, 0.0, f);

    m_frontLeftSwerveModule->SetTurningPosition(s * 360_deg);
    m_frontRightSwerveModule->SetTurningPosition(s * 360_deg);
    m_rearLeftSwerveModule->SetTurningPosition(s * 360_deg);
    m_rearRightSwerveModule->SetTurningPosition(s * 360_deg);
  }

  // void DrivePositionPID(double P, double I, double IZ, double IM, double D, double DF, double F);
  // void DriveVelocityPID(double P, double I, double IZ, double IM, double D, double DF, double F);
}

// The most general form of movement for a swerve is specified by thee vectors,
// at each wheel: X and Y velocity, and rotational velocity, about an arbitrary
// point in the XY plane.  Here, rotational velocity is about the center of the
// robot.  Of course, each of these velocities may be continually varied.  By
// adding a center of rotation (shared by all wheels), this could be made fully
// general.  This would likely come in through `kDriveKinematics` not being a
// preset constant.  XXX Actually, look at `ToSwerveModuleStates` -- bug here

// Another means of specifying movement is to point each swerve module from
// rest, then to specify a distance to translate.  In this mode, any rotation
// would normally be done distinct from translation.  However, it is also
// possible to specify a velocity for each drive motor, to use with unequal
// distances or to limit these velocities.  This provides an intuitive means of
// dead reckoning.  However, this may be approxomated simply by setting some of
// the parameters to zero then waiting for the sense outputs to settle near the
// desired values.

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                           bool fieldRelative) noexcept
{
  // XXX
  return;
  // XXX

  wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_ahrs->GetRotation2d())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.NormalizeWheelSpeeds(&states, 1_mps);

  SetModuleStates(states);
}

void DriveSubsystem::ResetEncoders() noexcept
{
  m_frontLeftSwerveModule->ResetEncoders();
  m_frontRightSwerveModule->ResetEncoders();
  m_rearLeftSwerveModule->ResetEncoders();
  m_rearRightSwerveModule->ResetEncoders();
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> &desiredStates) noexcept
{
  m_frontLeftSwerveModule->SetDesiredState(desiredStates[0]);
  m_frontRightSwerveModule->SetDesiredState(desiredStates[1]);
  m_rearLeftSwerveModule->SetDesiredState(desiredStates[2]);
  m_rearRightSwerveModule->SetDesiredState(desiredStates[3]);
}

units::degree_t DriveSubsystem::GetHeading() const noexcept
{
  double heading = 0.0;
  if (m_ahrs)
  {
    heading = m_ahrs->GetAngle(); // In degrees.
  }

  return units::degree_t{heading};
}

// XXX Do this here (is in AHRS), so can use GetHeading
//  Rotation2d GetRotation2d() const {
//    return Rotation2d{-GetHeading()};
//  }

void DriveSubsystem::ZeroHeading() noexcept
{
  if (m_ahrs)
  {
    m_ahrs->ZeroYaw();
  }
}

double DriveSubsystem::GetTurnRate() noexcept
{
  double rate = 0.0;
  if (m_ahrs)
  {
    rate = m_ahrs->GetRate(); // In degrees/second.
  }

  return rate;
}
