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
#include <frc2/command/CommandScheduler.h>
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
// XXX create "safe" wrapper for IMU and use here
  try
  {
    m_ahrs = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

    using namespace std::chrono_literals;

    const std::chrono::steady_clock::time_point wait_until = std::chrono::steady_clock::now() + 20s;
    while (!m_ahrs->IsConnected() || m_ahrs->IsCalibrating())
    {
      std::this_thread::sleep_for(100ms);
      // Upon timeout, go on and hope for the best; the driver can switch off
      // field-relative drive if there's a major problem.
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
#endif

  // Initial position (third parameter) defaulted to "frc::Pose2d()"; initial
  // angle (second parameter) is automatically zeroed by navX initialization.
  m_odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(kDriveKinematics, 0_deg);

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
  m_frontLeftSwerveModule->Periodic(m_run);
  m_frontRightSwerveModule->Periodic(m_run);
  m_rearLeftSwerveModule->Periodic(m_run);
  m_rearRightSwerveModule->Periodic(m_run);

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

  m_chooser = std::make_unique<frc::SendableChooser<frc2::Command *>>();

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
                            .WithWidget(frc::BuiltInWidgets::kGyro)
                            .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                                std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});
  m_frontRightTurning = &shuffleboardLayoutSwerveTurning.Add("Front Right", m_frontRightGyro)
                             .WithPosition(0, 1)
                             .WithWidget(frc::BuiltInWidgets::kGyro)
                             .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                                 std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});
  m_rearLeftTurning = &shuffleboardLayoutSwerveTurning.Add("Rear Left", m_rearLeftGyro)
                           .WithPosition(0, 1)
                           .WithWidget(frc::BuiltInWidgets::kGyro)
                           .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                               std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});
  m_rearRightTurning = &shuffleboardLayoutSwerveTurning.Add("Rear Right", m_rearRightGyro)
                            .WithPosition(1, 1)
                            .WithWidget(frc::BuiltInWidgets::kGyro)
                            .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                                std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});

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

  m_swerveRotation = &shuffleboardLayoutControls.Add("Rot +CCW", 0.0)
                          .WithPosition(0, 0)
                          .WithWidget(frc::BuiltInWidgets::kNumberBar);
  m_swerveX = &shuffleboardLayoutControls.Add("X +Forward", 0.0)
                   .WithPosition(1, 0)
                   .WithWidget(frc::BuiltInWidgets::kNumberBar);
  m_swerveY = &shuffleboardLayoutControls.Add("Y +Leftward", 0.0)
                   .WithPosition(1, 1)
                   .WithWidget(frc::BuiltInWidgets::kNumberBar);
  m_swerveStatus = &shuffleboardLayoutControls.Add("Status", false)
                        .WithPosition(0, 1)
                        .WithWidget(frc::BuiltInWidgets::kBooleanBox);
  m_driveLimit = &shuffleboardLayoutControls.Add("Drive Limit", 0.1)
                      .WithPosition(2, 0)
                      .WithWidget(frc::BuiltInWidgets::kNumberSlider)
                      .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                          std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                          std::make_pair("Max", nt::Value::MakeDouble(2.0))});
  m_displayMode = &shuffleboardLayoutControls.Add("Sense-Act", true)
                       .WithPosition(3, 0)
                       .WithWidget(frc::BuiltInWidgets::kToggleSwitch);
  m_swerveEnable = &shuffleboardLayoutControls.Add("Run-Stop", false)
                        .WithPosition(3, 1)
                        .WithWidget(frc::BuiltInWidgets::kToggleButton);
  m_commandChooser = &shuffleboardLayoutControls.Add("Command", *m_chooser)
                          .WithPosition(2, 1)
                          .WithWidget(frc::BuiltInWidgets::kComboBoxChooser);

  std::printf("OK.\n");
}

void DriveSubsystem::TestPeriodic() noexcept
{
  m_run = m_swerveEnable->GetEntry().GetBoolean(false);
  m_limit = m_driveLimit->GetEntry().GetDouble(0.0);

  m_frontLeftSwerveModule->TestPeriodic(!m_run);
  m_frontRightSwerveModule->TestPeriodic(!m_run);
  m_rearLeftSwerveModule->TestPeriodic(!m_run);
  m_rearRightSwerveModule->TestPeriodic(!m_run);

  // TestPeriodic() is not hooked into the scheduler, so normal Periodic() is
  // not called.  Do this here.
  Periodic();

  if (m_displayMode->GetEntry().GetBoolean(true))
  {
    // XXX Display commanded information
  }
  else
  {
    // XXX Display actual information (as below)
  }

  m_frontLeftGyro.Set((m_frontLeftSwerveModule->GetTurningPosition() + 180_deg) / 1_deg);
  m_frontRightGyro.Set((m_frontRightSwerveModule->GetTurningPosition() + 180_deg) / 1_deg);
  m_rearLeftGyro.Set((m_rearLeftSwerveModule->GetTurningPosition() + 180_deg) / 1_deg);
  m_rearRightGyro.Set((m_rearRightSwerveModule->GetTurningPosition() + 180_deg) / 1_deg);

  m_frontLeftDrive->GetEntry().SetDouble(m_frontLeftSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed);
  m_frontRightDrive->GetEntry().SetDouble(m_frontRightSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed);
  m_rearLeftDrive->GetEntry().SetDouble(m_rearLeftSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed);
  m_rearRightDrive->GetEntry().SetDouble(m_rearRightSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed);

  m_swerveStatus->GetEntry().SetBoolean(GetStatus());
  m_swerveRotation->GetEntry().SetDouble(m_rotation);
  m_swerveX->GetEntry().SetDouble(m_x);
  m_swerveY->GetEntry().SetDouble(m_y);

  if (m_turningPositionPIDController->GetE())
  {
    double p = m_turningPositionPIDController->GetP();
    double i = m_turningPositionPIDController->GetI();
    double d = m_turningPositionPIDController->GetD();
    double f = m_turningPositionPIDController->GetF();

    m_turningPositionPIDController->SetE(false);

    std::printf("**** Turning Position PID: ( %f / %f / %f / %f )\n", p, i, d, f);

    m_frontLeftSwerveModule->TurningPositionPID(p, i, pidf::kTurningPositionIM,
                                                pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_frontRightSwerveModule->TurningPositionPID(p, i, pidf::kTurningPositionIM,
                                                 pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_rearLeftSwerveModule->TurningPositionPID(p, i, pidf::kTurningPositionIM,
                                               pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_rearRightSwerveModule->TurningPositionPID(p, i, pidf::kTurningPositionIM,
                                                pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
  }

  if (m_drivePositionPIDController->GetE())
  {
    double p = m_drivePositionPIDController->GetP();
    double i = m_drivePositionPIDController->GetI();
    double d = m_drivePositionPIDController->GetD();
    double f = m_drivePositionPIDController->GetF();

    m_drivePositionPIDController->SetE(false);

    std::printf("**** Drive Position PID: ( %f / %f / %f / %f )\n", p, i, d, f);

    m_frontLeftSwerveModule->DrivePositionPID(p, i, pidf::kTurningPositionIM,
                                              pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_frontRightSwerveModule->DrivePositionPID(p, i, pidf::kTurningPositionIM,
                                               pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_rearLeftSwerveModule->DrivePositionPID(p, i, pidf::kTurningPositionIM,
                                             pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_rearRightSwerveModule->DrivePositionPID(p, i, pidf::kTurningPositionIM,
                                              pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
  }

  if (m_driveVelocityPIDController->GetE())
  {
    double p = m_driveVelocityPIDController->GetP();
    double i = m_driveVelocityPIDController->GetI();
    double d = m_driveVelocityPIDController->GetD();
    double f = m_driveVelocityPIDController->GetF();

    m_driveVelocityPIDController->SetE(false);

    std::printf("**** Drive Velocity PID: ( %f / %f / %f / %f )\n", p, i, d, f);

    m_frontLeftSwerveModule->DriveVelocityPID(p, i, pidf::kTurningPositionIM,
                                              pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_frontRightSwerveModule->DriveVelocityPID(p, i, pidf::kTurningPositionIM,
                                               pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_rearLeftSwerveModule->DriveVelocityPID(p, i, pidf::kTurningPositionIM,
                                             pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
    m_rearRightSwerveModule->DriveVelocityPID(p, i, pidf::kTurningPositionIM,
                                              pidf::kTurningPositionIZ, d, pidf::kTurningPositionDF, f);
  }

  // m_chooser->GetSelected()
  // XXX run chosen command in test mode
  // XXX if run, frc2::CommandScheduler::GetInstance().Run();
  // XXX if stop, frc2::CommandScheduler::GetInstance().CancelAll();
  // XXX when chosen, chosenCommand->Schedule();
}

bool DriveSubsystem::GetStatus() noexcept
{
  return m_frontLeftSwerveModule->GetStatus() &&
         m_frontRightSwerveModule->GetStatus() &&
         m_rearLeftSwerveModule->GetStatus() &&
         m_rearRightSwerveModule->GetStatus();
}

bool ZeroModules() noexcept { return true; }

bool SetTurnInPlace(bool) noexcept { return true; }

bool SetTurningPosition(const units::angle::degree_t position) noexcept { return true; }

bool SetDriveBrakeMode(bool brake) noexcept { return true; }

bool SetTurnByAngle(units::degree_t angle) noexcept { return true; }

bool SetDriveDistance(units::length::meter_t distance) noexcept { return true; }

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
  m_rotation = rot / physical::kMaxTurnRate;
  m_x = xSpeed / physical::kMaxDriveSpeed;
  m_y = ySpeed / physical::kMaxDriveSpeed;

  wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_ahrs->GetRotation2d()) // XXX Gyro might need negating
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.NormalizeWheelSpeeds(&states, physical::kMaxDriveSpeed);

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
  // m_run and m_limit are only used in Test Mode, by default they do not
  // modify anything here.  In Test Mode, they switch between control via the
  // Swerve tab or via the individual Swerve Module tabs.
  if (m_run)
  {
    auto [frontLeft, frontRight, rearLeft, rearRight] = desiredStates;

    frontLeft.speed *= m_limit;
    frontRight.speed *= m_limit;
    rearLeft.speed *= m_limit;
    rearRight.speed *= m_limit;

    m_frontLeftSwerveModule->SetDesiredState(frontLeft);
    m_frontRightSwerveModule->SetDesiredState(frontRight);
    m_rearLeftSwerveModule->SetDesiredState(rearLeft);
    m_rearRightSwerveModule->SetDesiredState(rearRight);
  }
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
    rate = -m_ahrs->GetRate(); // In degrees/second.
  }

  return rate;
}
