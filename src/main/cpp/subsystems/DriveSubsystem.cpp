


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// See https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html.
// See https://docs.wpilib.org/en/latest/docs/software/kinematics-and-odometry/swerve-drive-odometry.html.
// See https://github.com/wpilibsuite/allwpilib/blob/main/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/subsystems/DriveSubsystem.cpp.

#include "subsystems/DriveSubsystem.h"
#include "infrastructure/SwerveModule.h"

#include <frc/DataLogManager.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/array.h>
#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
//#include <frc/DriverStation.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <regex>
#include <span>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

using namespace pathplanner;

DriveSubsystem::DriveSubsystem() noexcept
{
  // Set up onboard printf-style logging.
  m_stringLog = wpi::log::StringLogEntry(frc::DataLogManager::GetLog(), "/DriveSubsystem/");

  // Set up the "navX" IMU first, so there's more time before it is used later.
  // See https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/.
  // Allow up to 20 seconds for callibration; it is supposed to be much faster,
  // when the IMU is kept still.  Consider using lights or other feedback so it
  // is very clear when this is occurring.
  DoSafeIMU("ctor", [&]() -> void
            {
    m_ahrs = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

    if (!m_ahrs)
    {
      throw std::runtime_error("m_ahrs");
    }

    {
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
    } });

  ZeroHeading();

  m_orientationController = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
      pidf::kDriveThetaP,
      pidf::kDriveThetaI,
      pidf::kDriveThetaD,
      std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
          pidf::kDriveThetaMaxVelocity,
          pidf::kDriveThetaMaxAcceleration}));

  m_orientationController->EnableContinuousInput(-180.0_deg, +180.0_deg);

  // This is precomputed and used in cases where there is close to a 180 degree
  // error; needed to get things moving in some cases (rotate 180 degrees).
  m_lagrange = m_orientationController->Calculate(+177.5_deg);
  m_orientationController->Reset(0.0_deg);

  // These are last, so there can be no movement from the swerve modules.
  m_frontLeftSwerveModule = std::make_unique<SwerveModule>(
      "Front Left",
      physical::kFrontLeftDriveMotorCanID,
      physical::kFrontLeftTurningMotorCanID,
      physical::kFrontLeftTurningEncoderPort,
      physical::kFrontLeftAlignmentOffset,
      physical::kLeftDriveMotorInverted);

  m_frontRightSwerveModule = std::make_unique<SwerveModule>(
      "Front Right",
      physical::kFrontRightDriveMotorCanID,
      physical::kFrontRightTurningMotorCanID,
      physical::kFrontRightTurningEncoderPort,
      physical::kFrontRightAlignmentOffset, 
      physical::kRightDriveMotorInverted);

  m_rearLeftSwerveModule = std::make_unique<SwerveModule>(
      "Rear Left",
      physical::kRearLeftDriveMotorCanID,
      physical::kRearLeftTurningMotorCanID,
      physical::kRearLeftTurningEncoderPort,
      physical::kRearLeftAlignmentOffset,
      physical::kLeftDriveMotorInverted);

  m_rearRightSwerveModule = std::make_unique<SwerveModule>(
      "Rear Right",
      physical::kRearRightDriveMotorCanID,
      physical::kRearRightTurningMotorCanID,
      physical::kRearRightTurningEncoderPort,
      physical::kRearRightAlignmentOffset,
      physical::kRightDriveMotorInverted);

  // Initial position (third parameter) defaulted to "frc::Pose2d()"; initial
  // angle (second parameter) is automatically zeroed by navX initialization.
  m_odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(kDriveKinematics, GetHeading(), GetModulePositions());
/*
  AutoBuilder::configureHolonomic(
      [this](){return GetPose();}, // Robot pose supplier
      [this](frc::Pose2d pose){ ResetOdometry(pose); }, // Method to reset odometry (will be called if your auto has a starting pose)
      [this](){ return GetSpeed(); }, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      [this](frc::ChassisSpeeds speeds){ Drive(speeds); },                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
      HolonomicPathFollowerConfig(       // HolonomicPathFollowerConfig, this should likely live in your Constants class
          PIDConstants(0.004, 0.0, 0.0), // Translation PID constants
          PIDConstants(0.006, 0.0, 0.0), // Rotation PID constants
          3.68808_mps,                   // Max module speed, in m/s
          0.4_m,                         // Drive base radius in meters. Distance from robot center to furthest module.
          ReplanningConfig()             // Default path replanning config. See the API for the options here
          ),
      []() {
        auto alliance = DriverStation::GetAlliance();
            if (alliance) {
                return alliance.value() == DriverStation::Alliance::kRed;
            }
        return false;
      },
      this // Reference to this subsystem to set requirements
  );
  */
}

std::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() noexcept
{
  return {
      m_frontLeftSwerveModule->GetPosition(),
      m_frontRightSwerveModule->GetPosition(),
      m_rearLeftSwerveModule->GetPosition(),
      m_rearRightSwerveModule->GetPosition()};
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

  DoSafeIMU("GetRotation2d()", [&]() -> void
            {
    if (m_ahrs)
    {
      botRot = -m_ahrs->GetRotation2d();
    }
    else
    {
      m_theta = 0.0;

      return;
    } });

  // Compute value to apply to correct robot orientation, or to follow rotation
  // profile.  Since things wrap, 180 degrees is a sort of Lagrange Point and
  // needs special handling, using precomputed theta.
  double theta = m_orientationController->Calculate(botRot.Degrees());
  units::angle::degree_t error = m_orientationController->GetPositionError();

  if (error > +177.5_deg)
  {
    theta = +m_lagrange;
  }
  else if (error < -177.5_deg)
  {
    theta = -m_lagrange;
  }

  if (theta > 0.0)
  {
    theta += m_thetaF;
  }
  else if (theta < 0.0)
  {
    theta -= m_thetaF;
  }
  m_theta = theta;

  m_odometry->Update(botRot, GetModulePositions());
}

frc::Pose2d DriveSubsystem::GetPose() noexcept
{
  return m_odometry->GetPose();
}

std::pair<double, double> DriveSubsystem::GetTilt() noexcept
{
  double x{0.0};
  double y{0.0};

  DoSafeIMU("GetPitch()/GetRoll()", [&]() -> void
            {
    x = m_ahrs->GetPitch();
    y = m_ahrs->GetRoll(); });

  return std::make_pair(x, y);
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) noexcept
{
  frc::Rotation2d botRot;

  DoSafeIMU("GetRotation2d()", [&]() -> void
            {
    if (m_ahrs)
    {
      botRot = -m_ahrs->GetRotation2d();
    } });

  m_odometry->ResetPosition(botRot, GetModulePositions(), pose);
}

void DriveSubsystem::CreateGraphTab(SwerveModule::GraphSelection graphSelection) noexcept
{
  // Only do this once.
  if (m_graph)
  {
    UpdateGraphTab(graphSelection);

    return;
  }

  std::vector<double> fourZerosVector{0.0, 0.0, 0.0, 0.0};
  std::span<double> fourZerosSpan{fourZerosVector.data(), fourZerosVector.size()};
  std::string path;

  frc::ShuffleboardTab &shuffleboardTab = frc::Shuffleboard::GetTab("PID Tuning (PAVF)");

  m_frontLeftGraph = &shuffleboardTab.Add("Front Left", fourZerosSpan)
                          .WithPosition(0, 0)
                          .WithSize(14, 6)
                          .WithWidget(frc::BuiltInWidgets::kGraph)
                          .WithProperties(wpi::StringMap<nt::Value>{
                              std::make_pair("Visible time", nt::Value::MakeDouble(5.0)),
                              std::make_pair("Unit", nt::Value::MakeString("")),
                              std::make_pair("X-axis auto scrolling", nt::Value::MakeBoolean(false))});

  path = m_frontLeftGraph->GetEntry()->GetTopic().GetName();
  path = std::regex_replace(path, std::regex("^\\/Shuffleboard\\/"), "/Shuffleboard/.metadata/");
  path += "/Properties/X-axis auto scrolling";
  m_frontLeftGraphScroll = path;

  m_frontRightGraph = &shuffleboardTab.Add("Front Right", fourZerosSpan)
                           .WithPosition(14, 0)
                           .WithSize(14, 6)
                           .WithWidget(frc::BuiltInWidgets::kGraph)
                           .WithProperties(wpi::StringMap<nt::Value>{
                               std::make_pair("Visible time", nt::Value::MakeDouble(5.0)),
                               std::make_pair("Unit", nt::Value::MakeString("")),
                               std::make_pair("X-axis auto scrolling", nt::Value::MakeBoolean(false))});

  path = m_frontRightGraph->GetEntry()->GetTopic().GetName();
  path = std::regex_replace(path, std::regex("^\\/Shuffleboard\\/"), "/Shuffleboard/.metadata/");
  path += "/Properties/X-axis auto scrolling";
  m_frontRightGraphScroll = path;

  m_rearLeftGraph = &shuffleboardTab.Add("Rear Left", fourZerosSpan)
                         .WithPosition(0, 6)
                         .WithSize(14, 6)
                         .WithWidget(frc::BuiltInWidgets::kGraph)
                         .WithProperties(wpi::StringMap<nt::Value>{
                             std::make_pair("Visible time", nt::Value::MakeDouble(5.0)),
                             std::make_pair("Unit", nt::Value::MakeString("")),
                             std::make_pair("X-axis auto scrolling", nt::Value::MakeBoolean(false))});

  path = m_rearLeftGraph->GetEntry()->GetTopic().GetName();
  path = std::regex_replace(path, std::regex("^\\/Shuffleboard\\/"), "/Shuffleboard/.metadata/");
  path += "/Properties/X-axis auto scrolling";
  m_rearLeftGraphScroll = path;

  m_rearRightGraph = &shuffleboardTab.Add("Rear Right", fourZerosSpan)
                          .WithPosition(14, 6)
                          .WithSize(14, 6)
                          .WithWidget(frc::BuiltInWidgets::kGraph)
                          .WithProperties(wpi::StringMap<nt::Value>{
                              std::make_pair("Visible time", nt::Value::MakeDouble(5.0)),
                              std::make_pair("Unit", nt::Value::MakeString("")),
                              std::make_pair("X-axis auto scrolling", nt::Value::MakeBoolean(false))});

  path = m_rearRightGraph->GetEntry()->GetTopic().GetName();
  path = std::regex_replace(path, std::regex("^\\/Shuffleboard\\/"), "/Shuffleboard/.metadata/");
  path += "/Properties/X-axis auto scrolling";
  m_rearRightGraphScroll = path;

  UpdateGraphTab(graphSelection);

  m_graph = true;
}

void DriveSubsystem::UpdateGraphTab(SwerveModule::GraphSelection graphSelection) noexcept
{
  if (m_graphSelection != SwerveModule::GraphSelection::kNone)
  {
    switch (m_graphSelection)
    {
    case SwerveModule::GraphSelection::kTurningRotation:
      std::printf("**** Turning Rotation Peak Values\n");
      break;
    case SwerveModule::GraphSelection::kDrivePosition:
      std::printf("**** Drive Position Peak Values\n");
      break;
    case SwerveModule::GraphSelection::kDriveVelocity:
      std::printf("**** Drive Velocity Peak Values\n");
      break;
    case SwerveModule::GraphSelection::kNone:
      break;
    }

    bool min = false;
    bool max = false;

    if (m_minProcessVariable != 0.0 || m_minProcessError != 0.0 || m_minProcessFirstDerivative != 0.0 || m_minProcessSecondDerivative != 0.0)
    {
      min = true;
      std::printf("**** MIN: ( %f / %f / %f / %f )\n", m_minProcessVariable, m_minProcessError, m_minProcessFirstDerivative, m_minProcessSecondDerivative);
    }
    if (m_maxProcessVariable != 0.0 || m_maxProcessError != 0.0 || m_maxProcessFirstDerivative != 0.0 || m_maxProcessSecondDerivative != 0.0)
    {
      max = true;
      std::printf("**** MAX: ( %f / %f / %f / %f )\n", m_maxProcessVariable, m_maxProcessError, m_maxProcessFirstDerivative, m_maxProcessSecondDerivative);
    }
    if (min && max)
    {
      m_maxProcessVariable = std::max(m_maxProcessVariable, std::abs(m_minProcessVariable));
      m_maxProcessError = std::max(m_maxProcessError, std::abs(m_minProcessError));
      m_maxProcessFirstDerivative = std::max(m_maxProcessFirstDerivative, std::abs(m_minProcessFirstDerivative));
      m_maxProcessSecondDerivative = std::max(m_maxProcessSecondDerivative, std::abs(m_minProcessSecondDerivative));

      std::printf("**** ABS: ( %f / %f / %f / %f )\n", m_maxProcessVariable, m_maxProcessError, m_maxProcessFirstDerivative, m_maxProcessSecondDerivative);
    }

    if (m_graph)
    {
      nt::NetworkTableEntry fr = nt::NetworkTableInstance::GetDefault().GetEntry(m_frontRightGraphScroll);
      nt::NetworkTableEntry fl = nt::NetworkTableInstance::GetDefault().GetEntry(m_frontLeftGraphScroll);
      nt::NetworkTableEntry rr = nt::NetworkTableInstance::GetDefault().GetEntry(m_rearRightGraphScroll);
      nt::NetworkTableEntry rl = nt::NetworkTableInstance::GetDefault().GetEntry(m_rearLeftGraphScroll);

      if (!fr.SetBoolean(m_run) || !fl.SetBoolean(m_run) || !rr.SetBoolean(m_run) || !rl.SetBoolean(m_run))
      {
        std::printf("**** Graph Scroll Control Error.\n");
      }
    }
  }

  m_graphSelection = graphSelection;
  m_minProcessVariable = 0.0;
  m_maxProcessVariable = 0.0;
  m_minProcessError = 0.0;
  m_maxProcessError = 0.0;
  m_minProcessFirstDerivative = 0.0;
  m_maxProcessFirstDerivative = 0.0;
  m_minProcessSecondDerivative = 0.0;
  m_maxProcessSecondDerivative = 0.0;
}

void DriveSubsystem::TestInit() noexcept
{
  m_run = false;
  m_limit = 0.1;

  UpdateGraphTab(SwerveModule::GraphSelection::kNone);

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

  frc::ShuffleboardTab &shuffleboardTab = frc::Shuffleboard::GetTab("Swerve");

  frc::ShuffleboardLayout &shuffleboardLayoutSwerveTurning =
      shuffleboardTab.GetLayout("Swerve Turning",
                                frc::BuiltInLayouts::kGrid)
          .WithPosition(0, 0)
          .WithSize(9, 13)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(2.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutPIDSettings =
      shuffleboardTab.GetLayout("PID Settings",
                                frc::BuiltInLayouts::kGrid)
          .WithPosition(9, 0)
          .WithSize(19, 7)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(1.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutSwerveDrive =
      shuffleboardTab.GetLayout("Swerve Drive",
                                frc::BuiltInLayouts::kGrid)
          .WithPosition(9, 7)
          .WithSize(7, 6)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(2.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

  frc::ShuffleboardLayout &shuffleboardLayoutControls =
      shuffleboardTab.GetLayout("Controls",
                                frc::BuiltInLayouts::kGrid)
          .WithPosition(16, 7)
          .WithSize(12, 6)
          .WithProperties(wpi::StringMap<nt::Value>{
              std::make_pair("Number of columns", nt::Value::MakeDouble(4.0)),
              std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

  m_frontLeftTurning = &shuffleboardLayoutSwerveTurning.Add("Front Left", m_frontLeftGyro)
                            .WithPosition(0, 0)
                            .WithWidget(frc::BuiltInWidgets::kGyro)
                            .WithProperties(wpi::StringMap<nt::Value>{
                                std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});
  m_frontRightTurning = &shuffleboardLayoutSwerveTurning.Add("Front Right", m_frontRightGyro)
                             .WithPosition(1, 0)
                             .WithWidget(frc::BuiltInWidgets::kGyro)
                             .WithProperties(wpi::StringMap<nt::Value>{
                                 std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});
  m_rearLeftTurning = &shuffleboardLayoutSwerveTurning.Add("Rear Left", m_rearLeftGyro)
                           .WithPosition(0, 1)
                           .WithWidget(frc::BuiltInWidgets::kGyro)
                           .WithProperties(wpi::StringMap<nt::Value>{
                               std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});
  m_rearRightTurning = &shuffleboardLayoutSwerveTurning.Add("Rear Right", m_rearRightGyro)
                            .WithPosition(1, 1)
                            .WithWidget(frc::BuiltInWidgets::kGyro)
                            .WithProperties(wpi::StringMap<nt::Value>{
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
                          .WithProperties(wpi::StringMap<nt::Value>{
                              std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                              std::make_pair("Max", nt::Value::MakeDouble(1.0))});
  m_frontRightDrive = &shuffleboardLayoutSwerveDrive.Add("Front Right", 0.0)
                           .WithPosition(1, 0)
                           .WithWidget(frc::BuiltInWidgets::kNumberBar)
                           .WithProperties(wpi::StringMap<nt::Value>{
                               std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                               std::make_pair("Max", nt::Value::MakeDouble(1.0))});
  m_rearLeftDrive = &shuffleboardLayoutSwerveDrive.Add("Rear Left", 0.0)
                         .WithPosition(0, 1)
                         .WithWidget(frc::BuiltInWidgets::kNumberBar)
                         .WithProperties(wpi::StringMap<nt::Value>{
                             std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                             std::make_pair("Max", nt::Value::MakeDouble(1.0))});
  m_rearRightDrive = &shuffleboardLayoutSwerveDrive.Add("Rear Right", 0.0)
                          .WithPosition(1, 1)
                          .WithWidget(frc::BuiltInWidgets::kNumberBar)
                          .WithProperties(wpi::StringMap<nt::Value>{
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
                      .WithProperties(wpi::StringMap<nt::Value>{
                          std::make_pair("Min", nt::Value::MakeDouble(0.0)),
                          std::make_pair("Max", nt::Value::MakeDouble(2.0))});
  m_displayMode = &shuffleboardLayoutControls.Add("Sense-Act", true)
                       .WithPosition(3, 0)
                       .WithWidget(frc::BuiltInWidgets::kToggleSwitch);
  m_swerveEnable = &shuffleboardLayoutControls.Add("Run-Stop", false)
                        .WithPosition(3, 1)
                        .WithWidget(frc::BuiltInWidgets::kToggleButton);

  m_commandChooser = &shuffleboardLayoutControls.Add("Command", m_chooser)
                          .WithPosition(2, 1)
                          .WithWidget(frc::BuiltInWidgets::kComboBoxChooser);

  std::printf("OK.\n");
}

void DriveSubsystem::TestExit() noexcept
{
  m_run = true;
  m_limit = 1.0;

  UpdateGraphTab(SwerveModule::GraphSelection::kNone);

  m_frontLeftSwerveModule->TestExit();
  m_frontRightSwerveModule->TestExit();
  m_rearLeftSwerveModule->TestExit();
  m_rearRightSwerveModule->TestExit();
}

void DriveSubsystem::TestPeriodic() noexcept
{
  const bool run = m_swerveEnable->GetEntry()->GetBoolean(false);

  m_limit = m_driveLimit->GetEntry()->GetDouble(0.1);

  if (run != m_run)
  {
    m_run = run;

    UpdateGraphTab(m_graphSelection);
  }
  m_run = run;

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

  if (m_displayMode->GetEntry()->GetBoolean(true))
  {
    // Display commanded information.
    m_frontLeftGyro.Set(m_commandedStateFrontLeft.angle.Degrees() / 1.0_deg);
    m_frontRightGyro.Set(m_commandedStateFrontRight.angle.Degrees() / 1.0_deg);
    m_rearLeftGyro.Set(m_commandedStateRearLeft.angle.Degrees() / 1.0_deg);
    m_rearRightGyro.Set(m_commandedStateRearRight.angle.Degrees() / 1.0_deg);

    m_frontLeftDrive->GetEntry()->SetDouble(m_commandedStateFrontLeft.speed / physical::kMaxDriveSpeed);
    m_frontRightDrive->GetEntry()->SetDouble(m_commandedStateFrontRight.speed / physical::kMaxDriveSpeed);
    m_rearLeftDrive->GetEntry()->SetDouble(m_commandedStateRearLeft.speed / physical::kMaxDriveSpeed);
    m_rearRightDrive->GetEntry()->SetDouble(m_commandedStateRearRight.speed / physical::kMaxDriveSpeed);
  }
  else
  {
    // Display actual information (as read by sensors).
    double frontLeftTurn = m_frontLeftSwerveModule->GetTurningPosition() / 1.0_deg;
    double frontRightTurn = m_frontRightSwerveModule->GetTurningPosition() / 1.0_deg;
    double rearLeftTurn = m_rearLeftSwerveModule->GetTurningPosition() / 1.0_deg;
    double rearRightTurn = m_rearRightSwerveModule->GetTurningPosition() / 1.0_deg;

    double frontLeftSpeed = m_frontLeftSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed;
    double frontRightSpeed = m_frontRightSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed;
    double rearLeftSpeed = m_rearLeftSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed;
    double rearRightSpeed = m_rearRightSwerveModule->GetDriveVelocity() / physical::kMaxDriveSpeed;

    // If velocity is negative, flip both heading and velocity.
    auto normalize = [](double &speed, double &turn) -> void
    {
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

    m_frontLeftDrive->GetEntry()->SetDouble(frontLeftSpeed);
    m_frontRightDrive->GetEntry()->SetDouble(frontRightSpeed);
    m_rearLeftDrive->GetEntry()->SetDouble(rearLeftSpeed);
    m_rearRightDrive->GetEntry()->SetDouble(rearRightSpeed);
  }

  m_swerveStatus->GetEntry()->SetBoolean(GetStatus());
  m_swerveRotation->GetEntry()->SetDouble(m_rotation);
  m_swerveX->GetEntry()->SetDouble(m_x);
  m_swerveY->GetEntry()->SetDouble(m_y);

  if (m_graphSelection != SwerveModule::GraphSelection::kNone)
  {
    std::vector<double> fourDatumsVector{0.0, 0.0, 0.0, 0.0};
    std::span<double> fourDatumsSpan{fourDatumsVector.data(), fourDatumsVector.size()};

    const auto fl = m_frontLeftSwerveModule->TestModeGraphData(m_graphSelection);
    const auto fr = m_frontRightSwerveModule->TestModeGraphData(m_graphSelection);
    const auto rl = m_rearLeftSwerveModule->TestModeGraphData(m_graphSelection);
    const auto rr = m_rearRightSwerveModule->TestModeGraphData(m_graphSelection);

    m_minProcessVariable = std::min({m_minProcessVariable, std::get<0>(fl), std::get<0>(fr), std::get<0>(rl), std::get<0>(rr)});
    m_maxProcessVariable = std::max({m_maxProcessVariable, std::get<0>(fl), std::get<0>(fr), std::get<0>(rl), std::get<0>(rr)});
    m_minProcessError = std::min({m_minProcessError, std::get<1>(fl), std::get<1>(fr), std::get<1>(rl), std::get<1>(rr)});
    m_maxProcessError = std::max({m_maxProcessError, std::get<1>(fl), std::get<1>(fr), std::get<1>(rl), std::get<1>(rr)});
    m_minProcessFirstDerivative = std::min({m_minProcessFirstDerivative, std::get<2>(fl), std::get<2>(fr), std::get<2>(rl), std::get<2>(rr)});
    m_maxProcessFirstDerivative = std::max({m_maxProcessFirstDerivative, std::get<2>(fl), std::get<2>(fr), std::get<2>(rl), std::get<2>(rr)});
    m_minProcessSecondDerivative = std::min({m_minProcessSecondDerivative, std::get<3>(fl), std::get<3>(fr), std::get<3>(rl), std::get<3>(rr)});
    m_maxProcessSecondDerivative = std::max({m_maxProcessSecondDerivative, std::get<3>(fl), std::get<3>(fr), std::get<3>(rl), std::get<3>(rr)});

    fourDatumsVector[0] = std::get<0>(fl);
    fourDatumsVector[1] = std::get<1>(fl);
    fourDatumsVector[2] = std::get<2>(fl);
    fourDatumsVector[3] = std::get<3>(fl);
    m_frontLeftGraph->GetEntry()->SetDoubleArray(fourDatumsSpan);

    fourDatumsVector[0] = std::get<0>(fr);
    fourDatumsVector[1] = std::get<1>(fr);
    fourDatumsVector[2] = std::get<2>(fr);
    fourDatumsVector[3] = std::get<3>(fr);
    m_frontRightGraph->GetEntry()->SetDoubleArray(fourDatumsSpan);

    fourDatumsVector[0] = std::get<0>(rl);
    fourDatumsVector[1] = std::get<1>(rl);
    fourDatumsVector[2] = std::get<2>(rl);
    fourDatumsVector[3] = std::get<3>(rl);
    m_rearLeftGraph->GetEntry()->SetDoubleArray(fourDatumsSpan);

    fourDatumsVector[0] = std::get<0>(rr);
    fourDatumsVector[1] = std::get<1>(rr);
    fourDatumsVector[2] = std::get<2>(rr);
    fourDatumsVector[3] = std::get<3>(rr);
    m_rearRightGraph->GetEntry()->SetDoubleArray(fourDatumsSpan);
  }

  if (m_turningPositionPIDController->GetE())
  {
    double p = m_turningPositionPIDController->GetP();
    double a = m_turningPositionPIDController->GetI();
    double v = m_turningPositionPIDController->GetD();
    double f = m_turningPositionPIDController->GetF();

    m_turningPositionPIDController->SetE(false);

    std::printf("**** Turning Position PID: ( %f / %f / %f / %f )\n", p, a, v, f);

    CreateGraphTab(SwerveModule::GraphSelection::kTurningRotation);

    m_frontLeftSwerveModule->TurningPositionPID(p, pidf::kTurningPositionI, pidf::kTurningPositionIZ, pidf::kTurningPositionIM,
                                                pidf::kTurningPositionD, pidf::kTurningPositionDF, f, v, a);
    m_frontRightSwerveModule->TurningPositionPID(p, pidf::kTurningPositionI, pidf::kTurningPositionIZ, pidf::kTurningPositionIM,
                                                 pidf::kTurningPositionD, pidf::kTurningPositionDF, f, v, a);
    m_rearLeftSwerveModule->TurningPositionPID(p, pidf::kTurningPositionI, pidf::kTurningPositionIZ, pidf::kTurningPositionIM,
                                               pidf::kTurningPositionD, pidf::kTurningPositionDF, f, v, a);
    m_rearRightSwerveModule->TurningPositionPID(p, pidf::kTurningPositionI, pidf::kTurningPositionIZ, pidf::kTurningPositionIM,
                                                pidf::kTurningPositionD, pidf::kTurningPositionDF, f, v, a);
  }

  if (m_drivePositionPIDController->GetE())
  {
    double p = m_drivePositionPIDController->GetP();
    double a = m_drivePositionPIDController->GetI();
    double v = m_drivePositionPIDController->GetD();
    double f = m_drivePositionPIDController->GetF();

    m_drivePositionPIDController->SetE(false);

    std::printf("**** Drive Position PID: ( %f / %f / %f / %f )\n", p, a, v, f);

    CreateGraphTab(SwerveModule::GraphSelection::kDrivePosition);

    m_frontLeftSwerveModule->DrivePositionPID(p, pidf::kDrivePositionI, pidf::kDrivePositionIZ, pidf::kDrivePositionIM,
                                              pidf::kDrivePositionD, pidf::kDrivePositionDF, f, v, a);
    m_frontRightSwerveModule->DrivePositionPID(p, pidf::kDrivePositionI, pidf::kDrivePositionIZ, pidf::kDrivePositionIM,
                                               pidf::kDrivePositionD, pidf::kDrivePositionDF, f, v, a);
    m_rearLeftSwerveModule->DrivePositionPID(p, pidf::kDrivePositionI, pidf::kDrivePositionIZ, pidf::kDrivePositionIM,
                                             pidf::kDrivePositionD, pidf::kDrivePositionDF, f, v, a);
    m_rearRightSwerveModule->DrivePositionPID(p, pidf::kDrivePositionI, pidf::kDrivePositionIZ, pidf::kDrivePositionIM,
                                              pidf::kDrivePositionD, pidf::kDrivePositionDF, f, v, a);
  }

  if (m_driveVelocityPIDController->GetE())
  {
    double p = m_driveVelocityPIDController->GetP();
    double a = m_driveVelocityPIDController->GetI();
    double v = m_driveVelocityPIDController->GetD();
    double f = m_driveVelocityPIDController->GetF();

    m_driveVelocityPIDController->SetE(false);

    std::printf("**** Drive Velocity PID: ( %f / %f / %f / %f )\n", p, a, v, f);

    CreateGraphTab(SwerveModule::GraphSelection::kDriveVelocity);

    m_frontLeftSwerveModule->DriveVelocityPID(p, pidf::kDriveVelocityI, pidf::kDriveVelocityIZ, pidf::kDriveVelocityIM,
                                              pidf::kDriveVelocityD, pidf::kDriveVelocityDF, f, v, a);
    m_frontRightSwerveModule->DriveVelocityPID(p, pidf::kDriveVelocityI, pidf::kDriveVelocityIZ, pidf::kDriveVelocityIM,
                                               pidf::kDriveVelocityD, pidf::kDriveVelocityDF, f, v, a);
    m_rearLeftSwerveModule->DriveVelocityPID(p, pidf::kDriveVelocityI, pidf::kDriveVelocityIZ, pidf::kDriveVelocityIM,
                                             pidf::kDriveVelocityD, pidf::kDriveVelocityDF, f, v, a);
    m_rearRightSwerveModule->DriveVelocityPID(p, pidf::kDriveVelocityI, pidf::kDriveVelocityIZ, pidf::kDriveVelocityIM,
                                              pidf::kDriveVelocityD, pidf::kDriveVelocityDF, f, v, a);
  }

  // Now, run any selected command.  First, if in low-level Test Mode, cancel
  // any running command and return, before setting up any new command.
  if (!m_run)
  {
    if (m_command)
    {
      m_command->Cancel();
    }
    m_command.reset();

    return;
  }

  std::function<frc2::CommandPtr()> commandFactory = m_chooser.GetSelected();

  // If a new command has been selected, cancel any old one and then schedule
  // the new one.
  if (m_commandFactory.target_type() != commandFactory.target_type() || m_commandFactory.target<frc2::CommandPtr()>() != commandFactory.target<frc2::CommandPtr()>())
  {
    if (m_command)
    {
      m_command->Cancel();
      m_command.reset();
    }

    if (m_commandFactory)
    {
      m_command = commandFactory();
    }
    m_commandFactory = commandFactory;

    if (m_command)
    {
      m_command->Schedule();
    }
  }
}

void DriveSubsystem::DisabledInit() noexcept
{
  m_run = false;

  UpdateGraphTab(m_graphSelection);
}

void DriveSubsystem::DisabledExit() noexcept
{
  m_run = true;

  UpdateGraphTab(m_graphSelection);
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
  m_theta = 0.0;

  m_frontLeftSwerveModule->ResetDrive();
  m_frontRightSwerveModule->ResetDrive();
  m_rearLeftSwerveModule->ResetDrive();
  m_rearRightSwerveModule->ResetDrive();

  m_orientationController->Reset(GetHeading(), units::angular_velocity::degrees_per_second_t{GetTurnRate()});
}

void DriveSubsystem::SetDriveBrakeMode(bool brake) noexcept
{
  m_frontLeftSwerveModule->SetDriveBrakeMode(brake);
  m_frontRightSwerveModule->SetDriveBrakeMode(brake);
  m_rearLeftSwerveModule->SetDriveBrakeMode(brake);
  m_rearRightSwerveModule->SetDriveBrakeMode(brake);
}

bool DriveSubsystem::ZeroModules() noexcept { return SetTurningPosition(0.0_deg); }

bool DriveSubsystem::SetTurnInPlace() noexcept
{
  m_rotation = 0.0;
  m_x = 0.0;
  m_y = 0.0;

  // Set all wheels tangent, at the given module.
  const std::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0.0_mps, 0.0_mps, physical::kMaxTurnRate});

  auto [frontLeft, frontRight, rearLeft, rearRight] = states;

  frontLeft.speed = 0.0_mps;
  frontRight.speed = 0.0_mps;
  rearLeft.speed = 0.0_mps;
  rearRight.speed = 0.0_mps;

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
  const std::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0.0_mps, 0.0_mps, physical::kMaxTurnRate});

  auto [frontLeft, frontRight, rearLeft, rearRight] = states;

  frontLeft.speed = 0.0_mps;
  frontRight.speed = 0.0_mps;
  rearLeft.speed = 0.0_mps;
  rearRight.speed = 0.0_mps;

  frontLeft.angle = frontLeft.angle + frc::Rotation2d(90.0_deg);
  frontRight.angle = frontRight.angle + frc::Rotation2d(90.0_deg);
  rearLeft.angle = rearLeft.angle + frc::Rotation2d(90.0_deg);
  rearRight.angle = rearRight.angle + frc::Rotation2d(90.0_deg);

  m_commandedStateFrontLeft = frontLeft;
  m_commandedStateFrontRight = frontRight;
  m_commandedStateRearLeft = rearLeft;
  m_commandedStateRearRight = rearRight;

  m_frontLeftSwerveModule->SetDesiredState(frontLeft);
  m_frontRightSwerveModule->SetDesiredState(frontRight);
  m_rearLeftSwerveModule->SetDesiredState(rearLeft);
  m_rearRightSwerveModule->SetDesiredState(rearRight);

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

  m_commandedStateFrontLeft.speed = 0.0_mps;
  m_commandedStateFrontRight.speed = 0.0_mps;
  m_commandedStateRearLeft.speed = 0.0_mps;
  m_commandedStateRearRight.speed = 0.0_mps;

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

bool DriveSubsystem::SetTurnToAngle(units::degree_t angle) noexcept
{
  m_orientationController->SetGoal(angle);

  if (!SetTurnInPlace())
  {
    return false;
  }

  // return SetDriveDistance(angle / 360.0_deg * physical::kDriveMetersPerTurningCircle);

  const units::meters_per_second_t linearVelocity = m_theta * physical::kMaxTurnRate / 360.0_deg * physical::kDriveMetersPerTurningCircle;

  m_frontLeftSwerveModule->SetDriveVelocity(linearVelocity);
  m_frontRightSwerveModule->SetDriveVelocity(linearVelocity);
  m_rearLeftSwerveModule->SetDriveVelocity(linearVelocity);
  m_rearRightSwerveModule->SetDriveVelocity(linearVelocity);

  return m_orientationController->AtGoal();
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
                           bool fieldRelative, units::meter_t x_center, units::meter_t y_center) noexcept
{
  m_rotation = rot / physical::kMaxTurnRate;
  m_x = xSpeed / physical::kMaxDriveSpeed;
  m_y = ySpeed / physical::kMaxDriveSpeed;

  frc::Rotation2d botRot;

  DoSafeIMU("GetRotation2d()", [&]() -> void
            {
    if (m_ahrs)
    {
      botRot = -m_ahrs->GetRotation2d();
    } });

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

  kDriveKinematics.DesaturateWheelSpeeds(&states, physical::kMaxDriveSpeed);

  SetModuleStates(states);
}

void DriveSubsystem::Drive(frc::ChassisSpeeds speeds)
{
  Drive(speeds.vx,speeds.vy,speeds.omega,false);
}

frc::ChassisSpeeds DriveSubsystem::GetSpeed()
{
  return kDriveKinematics.ToChassisSpeeds(
    {
    m_frontLeftSwerveModule->GetState(),
    m_frontRightSwerveModule->GetState(),
    m_rearLeftSwerveModule->GetState(),
    m_rearRightSwerveModule->GetState()
    
    }
  );
}
void DriveSubsystem::ResetEncoders() noexcept
{
  m_frontLeftSwerveModule->ResetEncoders();
  m_frontRightSwerveModule->ResetEncoders();
  m_rearLeftSwerveModule->ResetEncoders();
  m_rearRightSwerveModule->ResetEncoders();
}

void DriveSubsystem::SetModuleStates(std::array<frc::SwerveModuleState, 4> &desiredStates) noexcept
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
  if (frontLeft.speed == 0.0_mps &&
      frontRight.speed == 0.0_mps &&
      rearLeft.speed == 0.0_mps &&
      rearRight.speed == 0.0_mps)
  {
    m_frontLeftSwerveModule->SetDriveVelocity(0.0_mps);
    m_frontRightSwerveModule->SetDriveVelocity(0.0_mps);
    m_rearLeftSwerveModule->SetDriveVelocity(0.0_mps);
    m_rearRightSwerveModule->SetDriveVelocity(0.0_mps);

    return;
  }

  // m_limit is always unity, except in Test Mode.  So, by default, it does not
  // modify anything here.  In Test Mode, it can be used to slow things down.
  frontLeft.speed *= m_limit;
  frontRight.speed *= m_limit;
  rearLeft.speed *= m_limit;
  rearRight.speed *= m_limit;

  // To avoid brownout, reduce power when wheels are too far out of position.
  const units::angle::degree_t frontLeftTurningError = m_frontLeftSwerveModule->GetTurningPosition() - frontLeft.angle.Degrees();
  const units::angle::degree_t frontRightTurningError = m_frontRightSwerveModule->GetTurningPosition() - frontRight.angle.Degrees();
  const units::angle::degree_t rearLeftTurningError = m_rearLeftSwerveModule->GetTurningPosition() - rearLeft.angle.Degrees();
  const units::angle::degree_t rearRightTurningError = m_rearRightSwerveModule->GetTurningPosition() - rearRight.angle.Degrees();
  const double totalTurningError = std::abs(frontLeftTurningError.to<double>()) +
                                   std::abs(frontRightTurningError.to<double>()) +
                                   std::abs(rearLeftTurningError.to<double>()) +
                                   std::abs(rearRightTurningError.to<double>());

  if (totalTurningError > 10.0)
  {
    frontLeft.speed *= 0.5;
    frontRight.speed *= 0.5;
    rearLeft.speed *= 0.5;
    rearRight.speed *= 0.5;
  }

  m_frontLeftSwerveModule->SetDesiredState(frontLeft);
  m_frontRightSwerveModule->SetDesiredState(frontRight);
  m_rearLeftSwerveModule->SetDesiredState(rearLeft);
  m_rearRightSwerveModule->SetDesiredState(rearRight);
}

units::degree_t DriveSubsystem::GetHeading() noexcept
{
  units::degree_t heading{0.0};

  DoSafeIMU("GetAngle()", [&]() -> void
            {
    if (m_ahrs)
    {
      heading = units::degree_t{-m_ahrs->GetAngle()}; // In degrees already.
    } });

  return heading;
}

void DriveSubsystem::ZeroHeading() noexcept
{
  DoSafeIMU("ZeroYaw()", [&]() -> void
            {
    if (m_ahrs)
    {
      m_ahrs->ZeroYaw();
    } });
}

double DriveSubsystem::GetTurnRate() noexcept
{
  double rate{0.0};

  DoSafeIMU("GetRate()", [&]() -> void
            {
    if (m_ahrs)
    {
      rate = -m_ahrs->GetRate(); // In degrees/second (units not used in WPILib).
    } });

  return rate;
}

void DriveSubsystem::SysIdLogDrive(frc::sysid::SysIdRoutineLog *logger) noexcept
{
  m_frontLeftSwerveModule->SysIdLogDrive(logger);
  m_frontRightSwerveModule->SysIdLogDrive(logger);
  m_rearLeftSwerveModule->SysIdLogDrive(logger);
  m_rearRightSwerveModule->SysIdLogDrive(logger);
}

void DriveSubsystem::SysIdLogSteer(frc::sysid::SysIdRoutineLog *logger) noexcept
{
  m_frontLeftSwerveModule->SysIdLogSteer(logger);
  m_frontRightSwerveModule->SysIdLogSteer(logger);
  m_rearLeftSwerveModule->SysIdLogSteer(logger);
  m_rearRightSwerveModule->SysIdLogSteer(logger);
}

void DriveSubsystem::ThetaPID(double P, double I, double D, double F, double V, double A) noexcept
{
  m_thetaF = F;

  m_orientationController->SetPID(P, I, D);
  m_orientationController->SetConstraints(std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
      units::angular_velocity::degrees_per_second_t{V},
      units::angular_acceleration::degrees_per_second_squared_t{A}}));
}

void DriveSubsystem::BurnConfig() noexcept
{
  m_frontLeftSwerveModule->BurnConfig();
  m_frontRightSwerveModule->BurnConfig();
  m_rearLeftSwerveModule->BurnConfig();
  m_rearRightSwerveModule->BurnConfig();
}

void DriveSubsystem::ClearFaults() noexcept
{
  m_frontLeftSwerveModule->ClearFaults();
  m_frontRightSwerveModule->ClearFaults();
  m_rearLeftSwerveModule->ClearFaults();
  m_rearRightSwerveModule->ClearFaults();
}
