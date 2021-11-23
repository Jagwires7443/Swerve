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
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/ArrayRef.h>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

DriveSubsystem::DriveSubsystem() noexcept
{
  // Set up the "navX" IMU first, so there's more time before it is used later.
  // See https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/.
  // Allow up to 20 seconds for callibration; it is supposed to be much faster,
  // when the IMU is kept still.  Consider using lights or other feedback so it
  // is very clear when this is occurring.
  DoSafeIMU("ctor", [&]() -> void {
    m_ahrs = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

    if (!m_ahrs)
    {
      throw std::runtime_error("m_ahrs");
    }

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
  });

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

void DriveSubsystem::DoSafeIMU(const char *const what, std::function<void()> work) noexcept
{
  try
  {
    work();
  }
  catch (const std::exception &e)
  {
    m_ahrs = nullptr;

    std::printf("navX IMU %s exception: %s.\n", what, e.what());
  }
  catch (...)
  {
    m_ahrs = nullptr;

    std::printf("navX IMU %s unknown exception.\n", what);
  }
}

void DriveSubsystem::Periodic() noexcept
{
  m_frontLeftSwerveModule->Periodic();
  m_frontRightSwerveModule->Periodic();
  m_rearLeftSwerveModule->Periodic();
  m_rearRightSwerveModule->Periodic();

  frc::Rotation2d botRot;

  DoSafeIMU("GetRotation2d()", [&]() -> void {
    if (m_ahrs)
    {
      botRot = m_ahrs->GetRotation2d();
    }
  });

  m_odometry->Update(botRot, m_frontLeftSwerveModule->GetState(),
                     m_frontRightSwerveModule->GetState(), m_rearLeftSwerveModule->GetState(),
                     m_rearRightSwerveModule->GetState());
}

frc::Pose2d DriveSubsystem::GetPose() noexcept { return m_odometry->GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) noexcept
{
  frc::Rotation2d botRot;

  DoSafeIMU("GetRotation2d()", [&]() -> void {
    if (m_ahrs)
    {
      botRot = m_ahrs->GetRotation2d();
    }
  });

  m_odometry->ResetPosition(pose, botRot);
}

void DriveSubsystem::CreateGraphTab() noexcept
{
  // Only do this once.
  if (m_graph)
  {
    return;
  }

  m_graph = true;

  std::vector<double> fourZerosVector{0.0, 0.0, 0.0, 0.0};
  wpi::ArrayRef<double> fourZerosArrayRef(fourZerosVector);

  frc::ShuffleboardTab &shuffleboardTab = frc::Shuffleboard::GetTab("PID Tuning");

  m_frontLeftGraph = &shuffleboardTab.Add("Front Left", fourZerosArrayRef)
                          .WithPosition(0, 0)
                          .WithSize(14, 6)
                          .WithWidget(frc::BuiltInWidgets::kGraph)
                          .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                              std::make_pair("Unit", nt::Value::MakeString(""))});

  m_frontRightGraph = &shuffleboardTab.Add("Front Right", fourZerosArrayRef)
                           .WithPosition(14, 0)
                           .WithSize(14, 6)
                           .WithWidget(frc::BuiltInWidgets::kGraph)
                           .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                               std::make_pair("Unit", nt::Value::MakeString(""))});

  m_rearLeftGraph = &shuffleboardTab.Add("Rear Left", fourZerosArrayRef)
                         .WithPosition(0, 6)
                         .WithSize(14, 6)
                         .WithWidget(frc::BuiltInWidgets::kGraph)
                         .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                             std::make_pair("Unit", nt::Value::MakeString(""))});

  m_rearRightGraph = &shuffleboardTab.Add("Rear Right", fourZerosArrayRef)
                          .WithPosition(14, 6)
                          .WithSize(14, 6)
                          .WithWidget(frc::BuiltInWidgets::kGraph)
                          .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                              std::make_pair("Unit", nt::Value::MakeString(""))});
}

void DriveSubsystem::TestInit() noexcept
{
  m_run = false;
  m_limit = 0.1;
  m_graphSelection = SwerveModule::GraphSelection::kNone;

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
                             .WithPosition(1, 0)
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
                          .WithWidget(frc::BuiltInWidgets::kNumberBar)
                          .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                              std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                              std::make_pair("Max", nt::Value::MakeDouble(1.0))});
  m_frontRightDrive = &shuffleboardLayoutSwerveDrive.Add("Front Right", 0.0)
                           .WithPosition(1, 0)
                           .WithWidget(frc::BuiltInWidgets::kNumberBar)
                           .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                               std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                               std::make_pair("Max", nt::Value::MakeDouble(1.0))});
  m_rearLeftDrive = &shuffleboardLayoutSwerveDrive.Add("Rear Left", 0.0)
                         .WithPosition(0, 1)
                         .WithWidget(frc::BuiltInWidgets::kNumberBar)
                         .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                             std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                             std::make_pair("Max", nt::Value::MakeDouble(1.0))});
  m_rearRightDrive = &shuffleboardLayoutSwerveDrive.Add("Rear Right", 0.0)
                          .WithPosition(1, 1)
                          .WithWidget(frc::BuiltInWidgets::kNumberBar)
                          .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                              std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                              std::make_pair("Max", nt::Value::MakeDouble(1.0))});

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

void DriveSubsystem::TestExit() noexcept
{
  m_run = true;
  m_limit = 1.0;
  m_graphSelection = SwerveModule::GraphSelection::kNone;

  m_frontLeftSwerveModule->TestExit();
  m_frontRightSwerveModule->TestExit();
  m_rearLeftSwerveModule->TestExit();
  m_rearRightSwerveModule->TestExit();
}

void DriveSubsystem::TestPeriodic() noexcept
{
  const bool run = m_run;

  m_run = m_swerveEnable->GetEntry().GetBoolean(false);
  m_limit = m_driveLimit->GetEntry().GetDouble(0.1);

  m_frontLeftSwerveModule->TestModeControl(!m_run, m_testModeTurningVoltage, m_testModeDriveVoltage);
  m_frontRightSwerveModule->TestModeControl(!m_run, m_testModeTurningVoltage, m_testModeDriveVoltage);
  m_rearLeftSwerveModule->TestModeControl(!m_run, m_testModeTurningVoltage, m_testModeDriveVoltage);
  m_rearRightSwerveModule->TestModeControl(!m_run, m_testModeTurningVoltage, m_testModeDriveVoltage);

  // Test mode is not handled by the scheduler, so normal Periodic() is not
  // called in test mode; do this here.  It normaly runs before other code,
  // early in the main scheduling loop, every 20ms.
  Periodic();

  m_frontLeftSwerveModule->TestPeriodic();
  m_frontRightSwerveModule->TestPeriodic();
  m_rearLeftSwerveModule->TestPeriodic();
  m_rearRightSwerveModule->TestPeriodic();

  if (m_displayMode->GetEntry().GetBoolean(true))
  {
    // Display commanded information
    m_frontLeftGyro.Set(m_commandedStateFrontLeft.angle.Degrees() / 1_deg);
    m_frontRightGyro.Set(m_commandedStateFrontRight.angle.Degrees() / 1_deg);
    m_rearLeftGyro.Set(m_commandedStateRearLeft.angle.Degrees() / 1_deg);
    m_rearRightGyro.Set(m_commandedStateRearRight.angle.Degrees() / 1_deg);

    m_frontLeftDrive->GetEntry().SetDouble(m_commandedStateFrontLeft.speed / physical::kMaxDriveSpeed);
    m_frontRightDrive->GetEntry().SetDouble(m_commandedStateFrontRight.speed / physical::kMaxDriveSpeed);
    m_rearLeftDrive->GetEntry().SetDouble(m_commandedStateRearLeft.speed / physical::kMaxDriveSpeed);
    m_rearRightDrive->GetEntry().SetDouble(m_commandedStateRearRight.speed / physical::kMaxDriveSpeed);
  }
  else
  {
    // Display actual information (as read by sensors).
    double frontLeftTurn = m_frontLeftSwerveModule->GetTurningPosition() / 1_deg;
    double frontRightTurn = m_frontRightSwerveModule->GetTurningPosition() / 1_deg;
    double rearLeftTurn = m_rearLeftSwerveModule->GetTurningPosition() / 1_deg;
    double rearRightTurn = m_rearRightSwerveModule->GetTurningPosition() / 1_deg;

    double frontLeftSpeed = m_frontLeftSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed;
    double frontRightSpeed = m_frontRightSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed;
    double rearLeftSpeed = m_rearLeftSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed;
    double rearRightSpeed = m_rearRightSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed;

    // If velocity is negative, flip both heading and velocity.
    auto normalize = [](double &speed, double &turn) -> void {
      if (speed < 0)
      {
        speed *= -1.0;
        turn += 180.0;
        if (turn >= 180.0)
        {
          turn -= 360.0;
        }
      }
    };

    normalize(frontLeftSpeed, frontLeftTurn);
    normalize(frontRightSpeed, frontRightTurn);
    normalize(rearLeftSpeed, rearLeftTurn);
    normalize(rearRightSpeed, rearRightTurn);

    m_frontLeftGyro.Set(frontLeftTurn);
    m_frontRightGyro.Set(frontRightTurn);
    m_rearLeftGyro.Set(rearLeftTurn);
    m_rearRightGyro.Set(rearRightTurn);

    m_frontLeftDrive->GetEntry().SetDouble(frontLeftSpeed);
    m_frontRightDrive->GetEntry().SetDouble(frontRightSpeed);
    m_rearLeftDrive->GetEntry().SetDouble(rearLeftSpeed);
    m_rearRightDrive->GetEntry().SetDouble(rearRightSpeed);
  }

  m_swerveStatus->GetEntry().SetBoolean(GetStatus());
  m_swerveRotation->GetEntry().SetDouble(m_rotation);
  m_swerveX->GetEntry().SetDouble(m_x);
  m_swerveY->GetEntry().SetDouble(m_y);

  if (m_graphSelection != SwerveModule::GraphSelection::kNone)
  {
    CreateGraphTab();

    std::vector<double> fourDatumsVector{0.0, 0.0, 0.0, 0.0};
    wpi::ArrayRef<double> fourDatumsArrayRef(fourDatumsVector);

    const auto fl = m_frontLeftSwerveModule->TestModeGraphData(m_graphSelection);
    const auto fr = m_frontRightSwerveModule->TestModeGraphData(m_graphSelection);
    const auto rl = m_rearLeftSwerveModule->TestModeGraphData(m_graphSelection);
    const auto rr = m_rearRightSwerveModule->TestModeGraphData(m_graphSelection);

    // XXX
    std::printf("V: %f; A: %f\n", std::get<2>(fl), std::get<3>(fl));

    // fourDatumsVector[0] = std::get<0>(fl);
    // fourDatumsVector[1] = std::get<1>(fl);
    fourDatumsVector[2] = std::get<2>(fl);
    fourDatumsVector[3] = std::get<3>(fl);
    m_frontLeftGraph->GetEntry().SetDoubleArray(fourDatumsArrayRef);

    // fourDatumsVector[0] = std::get<0>(fr);
    // fourDatumsVector[1] = std::get<1>(fr);
    fourDatumsVector[2] = std::get<2>(fr);
    fourDatumsVector[3] = std::get<3>(fr);
    m_frontRightGraph->GetEntry().SetDoubleArray(fourDatumsArrayRef);

    // fourDatumsVector[0] = std::get<0>(rl);
    // fourDatumsVector[1] = std::get<1>(rl);
    fourDatumsVector[2] = std::get<2>(rl);
    fourDatumsVector[3] = std::get<3>(rl);
    m_rearLeftGraph->GetEntry().SetDoubleArray(fourDatumsArrayRef);

    // fourDatumsVector[0] = std::get<0>(rr);
    // fourDatumsVector[1] = std::get<1>(rr);
    fourDatumsVector[2] = std::get<2>(rr);
    fourDatumsVector[3] = std::get<3>(rr);
    m_rearRightGraph->GetEntry().SetDoubleArray(fourDatumsArrayRef);
  }

  if (m_turningPositionPIDController->GetE())
  {
    double p = m_turningPositionPIDController->GetP();
    double i = m_turningPositionPIDController->GetI();
    double d = m_turningPositionPIDController->GetD();
    double f = m_turningPositionPIDController->GetF();

    m_graphSelection = SwerveModule::GraphSelection::kTurningRotation;

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

    m_graphSelection = SwerveModule::GraphSelection::kDrivePosition;

    m_drivePositionPIDController->SetE(false);

    std::printf("**** Drive Position PID: ( %f / %f / %f / %f )\n", p, i, d, f);

    m_frontLeftSwerveModule->DrivePositionPID(p, i, pidf::kDrivePositionIM,
                                              pidf::kDrivePositionIZ, d, pidf::kDrivePositionDF, f,
                                              pidf::kDrivePositionMaxVelocity, pidf::kDrivePositionMaxAcceleration);
    m_frontRightSwerveModule->DrivePositionPID(p, i, pidf::kDrivePositionIM,
                                               pidf::kDrivePositionIZ, d, pidf::kDrivePositionDF, f,
                                               pidf::kDrivePositionMaxVelocity, pidf::kDrivePositionMaxAcceleration);
    m_rearLeftSwerveModule->DrivePositionPID(p, i, pidf::kDrivePositionIM,
                                             pidf::kDrivePositionIZ, d, pidf::kDrivePositionDF, f,
                                             pidf::kDrivePositionMaxVelocity, pidf::kDrivePositionMaxAcceleration);
    m_rearRightSwerveModule->DrivePositionPID(p, i, pidf::kDrivePositionIM,
                                              pidf::kDrivePositionIZ, d, pidf::kDrivePositionDF, f,
                                              pidf::kDrivePositionMaxVelocity, pidf::kDrivePositionMaxAcceleration);
  }

  if (m_driveVelocityPIDController->GetE())
  {
    double p = m_driveVelocityPIDController->GetP();
    double i = m_driveVelocityPIDController->GetI();
    double d = m_driveVelocityPIDController->GetD();
    double f = m_driveVelocityPIDController->GetF();

    m_graphSelection = SwerveModule::GraphSelection::kDriveVelocity;

    m_driveVelocityPIDController->SetE(false);

    std::printf("**** Drive Velocity PID: ( %f / %f / %f / %f )\n", p, i, d, f);

    m_frontLeftSwerveModule->DriveVelocityPID(p, i, pidf::kDriveVelocityIM,
                                              pidf::kDriveVelocityIZ, d, pidf::kDriveVelocityDF, f,
                                              pidf::kDriveVelocityMaxVelocity, pidf::kDriveVelocityMaxAcceleration);
    m_frontRightSwerveModule->DriveVelocityPID(p, i, pidf::kDriveVelocityIM,
                                               pidf::kDriveVelocityIZ, d, pidf::kDriveVelocityDF, f,
                                               pidf::kDriveVelocityMaxVelocity, pidf::kDriveVelocityMaxAcceleration);
    m_rearLeftSwerveModule->DriveVelocityPID(p, i, pidf::kDriveVelocityIM,
                                             pidf::kDriveVelocityIZ, d, pidf::kDriveVelocityDF, f,
                                             pidf::kDriveVelocityMaxVelocity, pidf::kDriveVelocityMaxAcceleration);
    m_rearRightSwerveModule->DriveVelocityPID(p, i, pidf::kDriveVelocityIM,
                                              pidf::kDriveVelocityIZ, d, pidf::kDriveVelocityDF, f,
                                              pidf::kDriveVelocityMaxVelocity, pidf::kDriveVelocityMaxAcceleration);
  }

  // Now, run any selected command.  First, if in low-level Test Mode, cancel
  // any running command and return, before setting up any new command.
  if (!m_run)
  {
    if (m_command)
    {
      m_command->Cancel();
    }
    m_command = nullptr;

    return;
  }

  frc2::Command *command = m_chooser->GetSelected();

  // If a new command has been selected, cancel any old one and then schedule
  // the new one.
  if (m_command != command)
  {
    if (m_command)
    {
      m_command->Cancel();
    }
    m_command = command;
    if (m_command)
    {
      m_command->Schedule(true);
    }
  }
}

bool DriveSubsystem::GetStatus() const noexcept
{
  return m_ahrs &&
         m_frontLeftSwerveModule->GetStatus() &&
         m_frontRightSwerveModule->GetStatus() &&
         m_rearLeftSwerveModule->GetStatus() &&
         m_rearRightSwerveModule->GetStatus();
}

void DriveSubsystem::ResetDrive() noexcept
{
  m_rotation = 0.0;
  m_x = 0.0;
  m_y = 0.0;

  m_frontLeftSwerveModule->ResetDrive();
  m_frontRightSwerveModule->ResetDrive();
  m_rearLeftSwerveModule->ResetDrive();
  m_rearRightSwerveModule->ResetDrive();
}

void DriveSubsystem::SetDriveBrakeMode(bool brake) noexcept
{
  m_frontLeftSwerveModule->SetDriveBrakeMode(brake);
  m_frontRightSwerveModule->SetDriveBrakeMode(brake);
  m_rearLeftSwerveModule->SetDriveBrakeMode(brake);
  m_rearRightSwerveModule->SetDriveBrakeMode(brake);
}

bool DriveSubsystem::ZeroModules() noexcept { return SetTurningPosition(0_deg); }

bool DriveSubsystem::SetTurnInPlace() noexcept
{
  m_rotation = 0.0;
  m_x = 0.0;
  m_y = 0.0;

  // Set all wheels tangent, at the given module.
  const wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0_mps, 0_mps, physical::kMaxTurnRate});

  auto [frontLeft, frontRight, rearLeft, rearRight] = states;

  frontLeft.speed = 0_mps;
  frontRight.speed = 0_mps;
  rearLeft.speed = 0_mps;
  rearRight.speed = 0_mps;

  m_commandedStateFrontLeft = frontLeft;
  m_commandedStateFrontRight = frontRight;
  m_commandedStateRearLeft = rearLeft;
  m_commandedStateRearRight = rearRight;

  m_frontLeftSwerveModule->SetTurningPosition(frontLeft.angle.Degrees());
  m_frontRightSwerveModule->SetTurningPosition(frontRight.angle.Degrees());
  m_rearLeftSwerveModule->SetTurningPosition(rearLeft.angle.Degrees());
  m_rearRightSwerveModule->SetTurningPosition(rearRight.angle.Degrees());

  const bool fl = m_frontLeftSwerveModule->CheckTurningPosition();
  const bool fr = m_frontRightSwerveModule->CheckTurningPosition();
  const bool rl = m_rearLeftSwerveModule->CheckTurningPosition();
  const bool rr = m_rearRightSwerveModule->CheckTurningPosition();

  return (fl && fr && rl && rr) || !m_run;
}

bool DriveSubsystem::SetLockWheelsX() noexcept
{
  m_rotation = 0.0;
  m_x = 0.0;
  m_y = 0.0;

  // Set all wheels at right angle to tangent, at the given module.  This forms
  // an "X", so the wheels resist being pushed (do not attempt to drive in this
  // configuration).
  const wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0_mps, 0_mps, physical::kMaxTurnRate});

  auto [frontLeft, frontRight, rearLeft, rearRight] = states;

  frontLeft.speed = 0_mps;
  frontRight.speed = 0_mps;
  rearLeft.speed = 0_mps;
  rearRight.speed = 0_mps;

  frontLeft.angle += frc::Rotation2d(90_deg);
  frontRight.angle += frc::Rotation2d(90_deg);
  rearLeft.angle += frc::Rotation2d(90_deg);
  rearRight.angle += frc::Rotation2d(90_deg);

  m_commandedStateFrontLeft = frontLeft;
  m_commandedStateFrontRight = frontRight;
  m_commandedStateRearLeft = rearLeft;
  m_commandedStateRearRight = rearRight;

  m_frontLeftSwerveModule->SetTurningPosition(frontLeft.angle.Degrees());
  m_frontRightSwerveModule->SetTurningPosition(frontRight.angle.Degrees());
  m_rearLeftSwerveModule->SetTurningPosition(rearLeft.angle.Degrees());
  m_rearRightSwerveModule->SetTurningPosition(rearRight.angle.Degrees());

  const bool fl = m_frontLeftSwerveModule->CheckTurningPosition();
  const bool fr = m_frontRightSwerveModule->CheckTurningPosition();
  const bool rl = m_rearLeftSwerveModule->CheckTurningPosition();
  const bool rr = m_rearRightSwerveModule->CheckTurningPosition();

  return (fl && fr && rl && rr) || !m_run;
}

bool DriveSubsystem::SetTurningPosition(const units::angle::degree_t position) noexcept
{
  m_rotation = 0.0;
  m_x = std::sin(units::angle::radian_t{position}.to<double>());
  m_y = std::cos(units::angle::radian_t{position}.to<double>());

  m_commandedStateFrontLeft.speed = 0_mps;
  m_commandedStateFrontRight.speed = 0_mps;
  m_commandedStateRearLeft.speed = 0_mps;
  m_commandedStateRearRight.speed = 0_mps;

  m_commandedStateFrontLeft.angle = frc::Rotation2d(position);
  m_commandedStateFrontRight.angle = frc::Rotation2d(position);
  m_commandedStateRearLeft.angle = frc::Rotation2d(position);
  m_commandedStateRearRight.angle = frc::Rotation2d(position);

  m_frontLeftSwerveModule->SetTurningPosition(position);
  m_frontRightSwerveModule->SetTurningPosition(position);
  m_rearLeftSwerveModule->SetTurningPosition(position);
  m_rearRightSwerveModule->SetTurningPosition(position);

  const bool fl = m_frontLeftSwerveModule->CheckTurningPosition();
  const bool fr = m_frontRightSwerveModule->CheckTurningPosition();
  const bool rl = m_rearLeftSwerveModule->CheckTurningPosition();
  const bool rr = m_rearRightSwerveModule->CheckTurningPosition();

  return (fl && fr && rl && rr) || !m_run;
}

bool DriveSubsystem::SetTurnByAngle(units::degree_t angle) noexcept
{
  return SetDriveDistance(angle / 360_deg * physical::kDriveMetersPerTurningCircle);
}

bool DriveSubsystem::SetDriveDistance(units::length::meter_t distance) noexcept
{
  m_frontLeftSwerveModule->SetDriveDistance(distance);
  m_frontRightSwerveModule->SetDriveDistance(distance);
  m_rearLeftSwerveModule->SetDriveDistance(distance);
  m_rearRightSwerveModule->SetDriveDistance(distance);

  const bool fl = m_frontLeftSwerveModule->CheckDriveDistance();
  const bool fr = m_frontRightSwerveModule->CheckDriveDistance();
  const bool rl = m_rearLeftSwerveModule->CheckDriveDistance();
  const bool rr = m_rearRightSwerveModule->CheckDriveDistance();

  return (fl && fr && rl && rr) || !m_run;
}

// The most general form of movement for a swerve is specified by thee vectors,
// at each wheel: X and Y velocity, and rotational velocity, about an arbitrary
// point in the XY plane.  By default, this point is the center of the robot.
// Of course, each of these velocities may be continually varied.  By
// specifying a center of rotation (shared by all wheels), this could be made
// fully general.

// Another means of specifying movement is to point each swerve module from
// rest, then to specify a distance to translate.  In this mode, any rotation
// would normally be done distinct from translation, providing a simple means
// of dead reckoning.  This involves periodically sending the same turning or
// drive command untile the commanded motion has been achieved.

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                           bool fieldRelative) noexcept
{
  Drive(xSpeed, ySpeed, rot, fieldRelative, 0_m, 0_m);
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                           bool fieldRelative, units::meter_t x_center, units::meter_t y_center) noexcept
{
  m_rotation = rot / physical::kMaxTurnRate;
  m_x = xSpeed / physical::kMaxDriveSpeed;
  m_y = ySpeed / physical::kMaxDriveSpeed;

  frc::Rotation2d botRot;

  DoSafeIMU("GetRotation2d()", [&]() -> void {
    if (m_ahrs)
    {
      botRot = m_ahrs->GetRotation2d();
    }
  });

  if (!m_ahrs)
  {
    fieldRelative = false;
  }

  // Center of rotation argument is defaulted to the center of the robot above,
  // but it is also possible to rotate about a different point.
  wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, botRot)
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
      frc::Translation2d(x_center, y_center));

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
  auto [frontLeft, frontRight, rearLeft, rearRight] = desiredStates;

  m_commandedStateFrontLeft = frontLeft;
  m_commandedStateFrontRight = frontRight;
  m_commandedStateRearLeft = rearLeft;
  m_commandedStateRearRight = rearRight;

  // Don't command turning if there is no drive; this is used from Drive(), and
  // it winds up causing the modules to all home to zero any time there is no
  // joystick input.  This check causes them to stay where they are, which is
  // no worse and saves energy, wear, and, potentially, time.  This is done
  // before applying m_limit (intentionally).
  if (frontLeft.speed == 0_mps &&
      frontRight.speed == 0_mps &&
      rearLeft.speed == 0_mps &&
      rearRight.speed == 0_mps)
  {
    return;
  }

  // m_limit is always unity, except in Test Mode.  So, by default, it does not
  // modify anything here.  In Test Mode, it can be used to slow things down.
  frontLeft.speed *= m_limit;
  frontRight.speed *= m_limit;
  rearLeft.speed *= m_limit;
  rearRight.speed *= m_limit;

  m_frontLeftSwerveModule->SetDesiredState(frontLeft);
  m_frontRightSwerveModule->SetDesiredState(frontRight);
  m_rearLeftSwerveModule->SetDesiredState(rearLeft);
  m_rearRightSwerveModule->SetDesiredState(rearRight);
}

units::degree_t DriveSubsystem::GetHeading() noexcept
{
  units::degree_t heading{0};

  DoSafeIMU("GetAngle()", [&]() -> void {
    if (m_ahrs)
    {
      heading = units::degree_t{-m_ahrs->GetAngle()}; // In degrees already.
    }
  });

  return heading;
}

void DriveSubsystem::ZeroHeading() noexcept
{
  DoSafeIMU("ZeroYaw()", [&]() -> void {
    if (m_ahrs)
    {
      m_ahrs->ZeroYaw();
    }
  });
}

double DriveSubsystem::GetTurnRate() noexcept
{
  double rate{0.0};

  DoSafeIMU("GetRate()", [&]() -> void {
    if (m_ahrs)
    {
      rate = -m_ahrs->GetRate(); // In degrees/second (units not used in WPILib).
    }
  });

  return rate;
}
