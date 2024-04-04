// XXX If no turning position, turn module into a caster (all coast)?
// XXX Update angle in disabled/periodic (for alignment).
// XXX Consider moving alignment API to degrees

// See https://github.com/wpilibsuite/allwpilib/blob/main/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/subsystems/SwerveModule.cpp.
// See https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control.

// A REV Robotics issue prevents running closed-loop control on the turning
// motor controller: https://trello.com/c/bvnPPcZD/108-add-continuous-pid-capability.
// The `m_rio` flag controls where this control is done, but the code is here
// to implement this either on the roboRIO or on the SPARK MAX.

#include "Constants.h"
#include "infrastructure/SwerveModule.h"

#include <frc/DataLogManager.h>
#include <frc/RobotController.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <networktables/NetworkTableValue.h>
#include <units/voltage.h>
#include <wpi/StringMap.h>

#include <bitset>
#include <cmath>
#include <cstdio>
#include <exception>
#include <ios>
#include <iomanip>
#include <sstream>
#include <string>

SwerveModule::SwerveModule(
    const char *const name,
    const int driveMotorCanID,
    const int turningMotorCanID,
    const int turningEncoderPort,
    const int alignmentOffset,
    const bool inverted) noexcept : m_name{name}, m_driveMotorInverted{inverted}
{
    // Set up onboard printf-style logging.
    std::string logName{"/SwerveModule/"};
    logName += name;
    m_stringLog = wpi::log::StringLogEntry(frc::DataLogManager::GetLog(), logName);

    std::printf("Swerve Module (%s) Initialization... ", m_name.c_str());

    // Construct turning position PID controller on the roboRIO; only used when
    // turning position control is running on the roborRIO.
    m_rioPIDController = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        pidf::kTurningPositionP,
        pidf::kTurningPositionI,
        pidf::kTurningPositionD,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            pidf::kTurningPositionMaxVelocity,
            pidf::kTurningPositionMaxAcceleration}));

    m_rioPIDController->EnableContinuousInput(-180.0_deg, +180.0_deg);

    // Construct turning absolute duty cycle encoder.  A `DigitalSource` is
    // required by the `DutyCycle` ctor.  Nothing here is expected to fail.
    // Note this has no explicit SetInverted() method; flip the turning motor
    // and it's incremental encoder in order to make things work out/line up.
    m_turningPositionPWM = std::make_unique<AngleSensor>(turningEncoderPort, alignmentOffset);

    // Motor controller configurations are only checked (or saved) in test mode
    // but a minimal amount is set up in these methods.
    m_turningMotorBase = SparkMaxFactory::CreateSparkMax(m_name + std::string(" Turning"), turningMotorCanID, m_turningMotorInverted);
    m_turningMotor = std::make_unique<SmartMotor<units::angle::degrees>>(*m_turningMotorBase);

    m_driveMotorBase = SparkMaxFactory::CreateSparkFlex(m_name + std::string(" Drive"), driveMotorCanID, m_driveMotorInverted);
    m_driveMotor = std::make_unique<SmartMotor<units::length::meters>>(*m_driveMotorBase);

    // kStatus1 includes velocity; kStatus2 includes position -- these are made
    // more frequent when graphing but may normally have a longer period.

    // Motor is in turns and turns/minute (RPM), do degrees and degrees/second.
    m_turningMotor->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint{250}}, // ms
        {"kStatus2", uint{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
    });
    m_turningMotor->ApplyConfig(false);

    // Motor is in turns and turns/minute (RPM), do meters and meters/second.
    m_driveMotor->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint{250}}, // ms
        {"kStatus2", uint{250}}, // ms
        {"kPositionConversionFactor", double{physical::kDriveMetersPerRotation}},
        {"kVelocityConversionFactor", double{physical::kDriveMetersPerRotation / 60.0}},
    });
    m_driveMotor->ApplyConfig(false);

    std::printf(" OK.\n");
}

// The error handling strategy for a swerve module, at present, is to treat all
// errors as fatal to the associated motor control, but to protect execution of
// the overall program from these errors.  Thus far, no errors have been seen
// and so it doesn't make sense to try to handle these.  In particular, it is
// possible that, subsequent to an error, there is some problem that will
// continue to cause errors.  Drivers are instructed on how to use the Driver
// Station to "Restart Robot Code", if there is a need to recover from an error
// that takes out a motor controller.  Not trying to recover simplifies things,
// and results in more pradictable behaviour in the face of uncertain
// conditions.  This way, there is less risk to keeping the rest of the robot
// working.  The rest of the code keeps on operatiing normally, isolating the
// problem motor controller.  In fact, this code will run without any motor
// controllers.

// One concern is a busy or electrically problematic CAN bus causing errors
// or timeouts.  Should this occur, the preference is to fix the root cause
// rather than trying to handle (and thereby hiding) the effects.

// Code which is closed source (and which does not specify "noexcept") is
// untrusted, in that it is surrounded by try/catch.  If a function returns any
// error, the code currently uses throw() to propagate the error.  It would not
// be hard to throw the error and catch fatal and non-fatal errors differently.
// But there has not been any need thus far.  The following four routines
// centralize the error handling logic.

// The motor controllers are the main failure points.  If the absolute position
// sensor stops working (likely a wiring issue), the code falls back to using
// the encoder on the turning motor/controller.  So, error handling is oriented
// around the two motor controllers.

// Construct and get/move turning motor objects.  It is possible for CAN or
// power issues to induce errors (particuarly from ctors), so do try/catch.
// The configuration is handled elsewhere; the expected case is that this has
// been saved, so that the motor controller comes up properly configured.

// This handles only those settings that are initially configured manually:
// kCanID (this cannot be configured programmatically), kMotorType/kSensorType,
// and kDataPortConfig; kIdleMode is not handled here).  To ensure improper
// inversions are never exposed, kAltEncoderInverted is handled here instead.

// Construct and get/move drive motor objects.  It is possible for CAN or
// power issues to induce errors (particuarly from ctors), so do try/catch.
// The configuration is handled elsewhere; the expected case is that this has
// been saved, so that the motor controller comes up properly configured.

// This handles only those settings that are initially configured manually:
// kCanID (this cannot be configured programmatically), kMotorType/kSensorType,
// and kDataPortConfig; kIdleMode is not handled here).  Since coast/brake is
// sometimes flipped, ensure things start out in coast.

bool SwerveModule::GetStatus() const noexcept
{
    return m_absoluteSensorGood &&
           m_turningMotorControllerValidated &&
           m_driveMotorControllerValidated;
}

// This automatically runs (at 20Hz).  It is called before any commands are
// scheduled, which is where other logic is run.  So the sequence is this
// method runs, other methods may be called, delay (before next iteration).

// For a swerve module, there is interplay between the turning position and the
// drive motor.  Logic here (and in the SetDrive*() methods below) ensures that
// the drive motor coasts with no applied power unless the turning position has
// reached the commanded position (within a specified tolerance).  This reduces
// treadwear, saves power, and helps the robot to drive more smoothly.  This is
// not something that can be handed off to an individual motor controller, so
// this Periodic() method is needed, even when the motor controllers are both
// doing closed-loop control.  Note that this behavior is particuarly helpful
// when battery voltage is low, since it ensures the turning motors get all the
// available power to turn before the drive motors recieve power.
void SwerveModule::Periodic() noexcept
{
    m_turningPositionPWM->Periodic();
    m_turningMotor->Periodic();
    m_driveMotor->Periodic();

    const bool priorTurningPositionAsCommanded = m_turningPositionAsCommanded;

    // CheckTurningPosition() updates all data members related to turning state
    // as a side effect.  This check passes when the swerve module has either
    // rotated into or out of the desired alignment.  For SPARK MAX, position
    // and velocity come in via periodic status frames and the APIs that return
    // these simply report the most recently recieved values.  So, some of this
    // data is sampled in this way.  The absolute position sensor has less of
    // this and is more continuous.  Note that the Set*() methods do not do
    // anything while in low-level test mode.
    m_turningPositionAsCommanded = CheckTurningPosition();
    if (priorTurningPositionAsCommanded != m_turningPositionAsCommanded)
    {
        // Turning now aligned as commanded, so power (or brake) drive motor.
        // Or turning no longer aligned as commanded, so coast drive motor.
        // Either way, these methods will sort things out.
        if (m_distanceVelocityNot)
        {
            SetDriveDistance(m_commandedDistance);
        }
        else
        {
            SetDriveVelocity(m_commandedVelocity);
        }
    }

    // If turning position PID is being done on the motor controller, there's
    // no more to do.  Even if PID is being done on the RIO, don't do this if
    // low-level test mode is controlling the motors.
    if (!m_rio || m_testModeControl || m_testModeTurningVoltage != 0.0)
    {
        return;
    }

    // Update (and apply below) turning position PID.
    double calculated = m_rioPIDController->Calculate(m_turningPosition);

    // Feedforward is a form of open-loop control.  For turning, there is not
    // much to do, but add in a constant value based only on the direction of
    // any error.  This is essentially how much output is needed to get going
    // and can help to compensate for friction.  At present, no deadband is
    // applied here.
    if (calculated > 0.0)
    {
        calculated += m_rioPID_F;
    }
    else if (calculated < 0.0)
    {
        calculated -= m_rioPID_F;
    }

    // Use voltage compensation, to offset low battery voltage.
    m_turningMotor->SetVoltage(calculated * 12.0_V);
}

void SwerveModule::ResetTurning() noexcept
{
    const std::optional<units::angle::degree_t> position = m_turningPositionPWM->GetAbsolutePosition();

    // If absolute position isn't available, there's no basis for resetting and
    // it's better to just let any old data stand.  This is when manual
    // pointing of the modules before a match could ensure a reasonable zero.
    // Of course, if anything is reset, etc. all bets are off -- still, leaving
    // things alone is the best that can be done under these circumstances.
    if (!position.has_value())
    {
        return;
    }

    m_turningPosition = position.value();
    m_turningMotor->SpecifyPosition(m_turningPosition);

    m_rioPIDController->Reset(m_turningPosition);
}

void SwerveModule::ResetDrive() noexcept
{
    m_driveMotor->SpecifyPosition(0.0_m);
}

units::angle::degree_t SwerveModule::GetTurningPosition() noexcept
{
    const std::optional<units::angle::degree_t> position = m_turningPositionPWM->GetAbsolutePosition();

    if (position.has_value())
    {
        m_turningPosition = position.value();

        return m_turningPosition;
    }

    // Absolute encoder is strongly preferred -- it has very low latency,
    // and is absolute.  Since the same encoder is used with the SPARK MAX,
    // the resolution and direct measurement of turning angle are identical
    // (so long as the incremental encoder was successfully reset at some
    // point and there has been no reset or similar problem).  But, this is
    // the best that can be done here.
    units::angle::degree_t encoderPosition = m_turningMotor->GetPosition();

    while (encoderPosition < 180.0_deg)
    {
        encoderPosition += 360.0_deg;
    }
    while (encoderPosition >= 180.0_deg)
    {
        encoderPosition -= 360.0_deg;
    }

    m_turningPosition = encoderPosition;

    return m_turningPosition;
}

void SwerveModule::SetTurningPosition(const units::angle::degree_t position) noexcept
{
    units::angle::degree_t adjustedPosition = position;

    // Defensive coding; could use fmod(adjustedPosition, 360_deg) here, except
    // this would still require some logic because of sign and should never run
    // in any case.
    while (adjustedPosition < -180.0_deg)
    {
        adjustedPosition += 360.0_deg;
    }
    while (adjustedPosition >= +180.0_deg)
    {
        adjustedPosition -= 360.0_deg;
    }

    m_commandedHeading = adjustedPosition;

    m_rioPIDController->SetGoal(adjustedPosition);

    if (m_rio || m_testModeControl || m_testModeTurningVoltage != 0.0)
    {
        return;
    }

    m_turningMotor->SeekPosition(adjustedPosition);
}

bool SwerveModule::CheckTurningPosition(const units::angle::degree_t tolerance) noexcept
{
    units::angle::degree_t error = GetTurningPosition() - m_commandedHeading;

    if (error < -180.0_deg)
    {
        error += 360.0_deg;
    }
    else if (error >= 180.0_deg)
    {
        error -= 360.0_deg;
    }

    return error >= -tolerance && error < tolerance;
}

// Drive position and velocity are in rotations and rotations/second,
// respectively.  Brake/coast also applies here; brake only in distance mode,
// once distance has been reached or in velocity mode, if commanded velocity is
// zero.  If the swerve module is not aligned, always coast, with no power.

// It would be possible to use SetPositionConversionFactor() so that distance
// is in dimensionless units matching meters, and SetVelocityConversionFactor()
// so that velocity is in dimensionless units matching meters per second, but
// this is handled here instead.  These conversions factor in gear/belt ratios,
// plus the circumference of the wheel (diameter * pi).  This yields units of
// meters and meters/second.  The conversion factor is determined empirically
// and represents rotations per meter or, (fractional) meters per rotation.

units::length::meter_t SwerveModule::GetDriveDistance() noexcept
{
    return m_driveMotor->GetPosition();
}

void SwerveModule::SetDriveDistance(units::length::meter_t distance) noexcept
{
    m_distanceVelocityNot = true;
    m_commandedDistance = distance;
    m_commandedVelocity = 0.0_mps;

    if (m_testModeControl || m_testModeDriveVoltage != 0.0)
    {
        return;
    }

    if (m_turningPositionAsCommanded)
    {
        // SetReference(SetPoint, rev::ControlType::kSmartMotion, 0, FeedForward)
        // XXX Use m_drivePosition_F here?
        m_driveMotor->SeekPosition(distance);
    }
    else
    {
        m_driveMotor->Stop();
    }
}

bool SwerveModule::CheckDriveDistance(const units::length::meter_t tolerance) noexcept
{
    if (!m_distanceVelocityNot)
    {
        return false;
    }

    const units::length::meter_t error = GetDriveDistance() - m_commandedDistance;

    return error >= -tolerance && error < tolerance;
}

units::velocity::meters_per_second_t SwerveModule::GetDriveVelocity() noexcept
{
    return m_driveMotor->GetVelocity();
}

void SwerveModule::SetDriveVelocity(units::velocity::meters_per_second_t velocity) noexcept
{
    m_distanceVelocityNot = false;
    m_commandedDistance = 0.0_m;
    m_commandedVelocity = velocity;

    if (m_testModeControl || m_testModeDriveVoltage != 0.0)
    {
        return;
    }

    const units::angle::degree_t angleError = m_turningPosition - m_commandedHeading;
    const double vectorAlignment = std::cos(units::angle::radian_t{angleError}.to<double>());

#if 0
    m_driveMotor->SeekVelocity(velocity * vectorAlignment);
#else
    m_driveMotor->SetVoltage(velocity * vectorAlignment / physical::kMaxDriveSpeed * 24.0_V);
#endif
}

const frc::SwerveModuleState SwerveModule::GetState() noexcept
{
    frc::SwerveModuleState result;

    result.angle = frc::Rotation2d(GetTurningPosition());
    result.speed = GetDriveVelocity();

    return result;
}

const frc::SwerveModulePosition SwerveModule::GetPosition() noexcept
{
    frc::SwerveModulePosition result;

    result.angle = frc::Rotation2d(GetTurningPosition());
    result.distance = GetDriveDistance();

    return result;
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &referenceState) noexcept
{
    // Optimize the reference state, to avoid spinning further than 90 degrees.
    frc::SwerveModuleState state = referenceState;

    const std::optional<units::angle::degree_t> position = m_turningPositionPWM->GetAbsolutePosition();

    if (position.has_value())
    {
        m_turningPosition = position.value();
        state = frc::SwerveModuleState::Optimize(referenceState, frc::Rotation2d(m_turningPosition));
    }

    SetTurningPosition(state.angle.Degrees());

    SetDriveVelocity(state.speed);
}

void SwerveModule::ResetEncoders() noexcept
{
    ResetTurning();
    ResetDrive();
}



void SwerveModule::SysIdLogDrive(frc::sysid::SysIdRoutineLog *logger) noexcept
{
    logger->Motor(m_driveMotor->GetName())
        .voltage(m_driveMotor->GetVoltage())
        .current(m_driveMotor->GetCurrent())
        .position(m_driveMotor->GetPosition())
        .velocity(m_driveMotor->GetVelocity());
}

void SwerveModule::SysIdLogSteer(frc::sysid::SysIdRoutineLog *logger) noexcept
{
    // Use the external/absolute position sensor for turning data.  Given a
    // target velocity of zero, the velocity should be the same as the error.
    units::angle::turn_t turning_position = m_turningPosition;
    units::angular_velocity::turns_per_second_t turning_velocity = m_rioPIDController->GetVelocityError();

    logger->Motor(m_turningMotor->GetName())
        .voltage(m_turningMotor->GetVoltage())
        .current(m_turningMotor->GetCurrent())
        .position(turning_position)
        .velocity(turning_velocity);
}


// This sets up the test mode shuffleboard tab; everything is specified
// programmatically.
void SwerveModule::TestInit() noexcept
{
    ResetEncoders();

    m_testModeControl = true;
    m_turningMotorControllerValidated = false;
    m_driveMotorControllerValidated = false;

    std::printf("Swerve Module (%s) Test Mode Init... ", m_name.c_str());

    // It would be very nice if Shufflboard allowed specifying properties here:
    // "gridSize": 32.0
    // "showGrid": false
    // "hgap": 0.0
    // "vgap": 0.0
    // As it is, best results will be obtained when the defaults for new tabs
    // are set in this way, except that the defaults do not seem to be applied.
    frc::ShuffleboardTab &shuffleboardTab = frc::Shuffleboard::GetTab(m_name);

    // Setup three grids for groupings of individual widgets.

    frc::ShuffleboardLayout &shuffleboardLayoutTurningPosition =
        shuffleboardTab.GetLayout("Turning Position",
                                  frc::BuiltInLayouts::kGrid)
            .WithPosition(0, 0)
            .WithSize(8, 13)
            .WithProperties(wpi::StringMap<nt::Value>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(3.0))});

    frc::ShuffleboardLayout &shuffleboardLayoutTurningMotor =
        shuffleboardTab.GetLayout("Turning Motor",
                                  frc::BuiltInLayouts::kGrid)
            .WithPosition(8, 0)
            .WithSize(20, 6)
            .WithProperties(wpi::StringMap<nt::Value>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(6.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

    frc::ShuffleboardLayout &shuffleboardLayoutDriveMotor =
        shuffleboardTab.GetLayout("Drive Motor",
                                  frc::BuiltInLayouts::kGrid)
            .WithPosition(8, 7)
            .WithSize(20, 6)
            .WithProperties(wpi::StringMap<nt::Value>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(6.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

    // Setup widgets.  XXX add fcn parameters
    m_turningPositionPWM->ShuffleboardCreate(
        shuffleboardLayoutTurningPosition,
        [&]() -> std::pair<units::angle::degree_t, units::angle::degree_t>
        { return std::make_pair(m_commandedHeading, m_turningMotor->GetPosition()); });

    m_turningMotor->ShuffleboardCreate(
        shuffleboardLayoutTurningMotor,
        [&](double control) -> void
        { m_turningControlUI = control; },
        [&]() -> void
        { m_turningControlUI = 0.0; m_turningResetUI = true; });

    m_driveMotor->ShuffleboardCreate(
        shuffleboardLayoutDriveMotor,
        [&](double control) -> void
        { m_driveControlUI = control; },
        [&]() -> void
        { m_driveControlUI = 0.0; m_driveResetUI = true; });

    std::printf(" OK.\n");
}

void SwerveModule::TestExit() noexcept
{
    m_testModeControl = false;
    m_turningMotorControllerValidated = true;
    m_driveMotorControllerValidated = true;

    // Robot is likely not enabled at this stage, but go ahead and command turn
    // to home position.  This will take effect when enabled.
    SetTurningPosition(0.0_deg);
}

// This updates data in the Shuffleboard tab and handles HMI interaction.
void SwerveModule::TestPeriodic() noexcept
{
    // Read controls information from Shuffleboard and manage interactive UI.
    bool zeroTurning = m_turningResetUI;
    double setTurning = m_turningControlUI;
    bool zeroDrive = m_driveResetUI;
    double setDrive = m_driveControlUI;

    m_turningResetUI = false;
    m_driveResetUI = false;

    // Check motor controller configuration, but only infrequently (and not when graphing)!
    const std::chrono::time_point now = std::chrono::steady_clock::now();
    if (m_graphSelection != GraphSelection::kNone)
    {
        m_turningMotorControllerValidated = true;
        m_driveMotorControllerValidated = true;
    }
    else if (now >= m_verifyMotorControllersWhen)
    {
        using namespace std::chrono_literals;

        m_verifyMotorControllersWhen = now + 15s;

        m_turningMotorControllerValidated = m_turningMotor->GetStatus();
        m_driveMotorControllerValidated = m_driveMotor->GetStatus();

        m_turningMotor->CheckConfig();
        m_driveMotor->CheckConfig();
    }

    // [-2048, +2048)
    std::optional<int> position = m_turningPositionPWM->GetAbsolutePositionWithoutAlignment();

    // This provides a rough means of zeroing the turning position.
    if (position.has_value())
    {
        if (zeroTurning)
        {
            // Work out new alignment so position becomes zero.
            int alignmentOffset = -position.value();
            if (alignmentOffset == +2048)
            {
                alignmentOffset = -2048;
            }

            m_turningPositionPWM->SetAlignment(alignmentOffset);
            position = 0;
            m_turningPosition = 0.0_deg;
        }
        else
        {
            // Alignment is [-2048, +2048).
            position = position.value() + m_turningPositionPWM->GetAlignment();
            if (position > 2047)
            {
                position = position.value() - 4096;
            }
            if (position < -2048)
            {
                position = position.value() + 4096;
            }
            m_turningPosition = position.value() / 2048.0 * 180.0_deg;
        }
    }

    if (zeroTurning)
    {
        // Logic above ensures that `position` is now zero; reset the
        // turning motor controller encoder to reflect this.
        m_turningMotor->SpecifyPosition(0.0_deg);
    }

    if (m_testModeControl)
    {
        m_turningMotor->Set(setTurning);
    }
    else if (m_testModeTurningVoltage != 0.0)
    {
        m_turningMotor->SetVoltage(m_testModeTurningVoltage * 1.0_V);
    }

    if (zeroDrive)
    {
        m_driveMotor->SpecifyPosition(0.0_m);
    }

    if (m_testModeControl)
    {
        m_driveMotor->Set(setDrive);
    }
    else if (m_testModeDriveVoltage != 0.0)
    {
        m_driveMotor->SetVoltage(m_testModeDriveVoltage * 1.0_V);
    }

    // Compute the discrepancy between the absolute encoder and the incremental
    // encoder (as seen by the motor controller).  This should be very small if
    // the module is not turning as the data is being collected.  However, if
    // the module is turning, some discrepany is expected.  For some of these
    // parameters, it may be useful to graph values over time, as the module is
    // rotated at constant angular velocity.  This would make it easy to find
    // the min/max or to spot any discontinuities, for example.  Also, computes
    // the error in achieving the commanded turning position (useful when
    // operating in closed-loop).

    if (m_graphSelection == GraphSelection::kNone)
    {
        m_lastFPGATime = 0;
        m_processVariable = 0.0;
        m_processError = 0.0;
        m_processFirstDerivative = 0.0;
        m_processSecondDerivitive = 0.0;
    }
    else
    {
        // In microseconds.
        uint64_t FPGATime = frc::RobotController::GetFPGATime();

        // Seconds.
        double deltaTime{(FPGATime - m_lastFPGATime) / 1000000.0};

        if (m_lastFPGATime == 0)
        {
            deltaTime = 0.0;
        }
        m_lastFPGATime = FPGATime;

        double deltaProcessError = m_processError;
        double deltaFirstDerivative = m_processFirstDerivative;

        switch (m_graphSelection)
        {
        case GraphSelection::kTurningRotation:
            if (position.has_value())
            {
                // Normalize to [-1, +1).
                m_processVariable = static_cast<double>(position.value()) / 2048.0;

                // Normalize to [-1, +1).
                const double commandedHeading = m_commandedHeading / 180.0_deg;
                const double counterclockwiseError = m_processVariable - commandedHeading;
                const double clockwiseError = commandedHeading - m_processVariable;

                if (std::abs(counterclockwiseError) < std::abs(clockwiseError))
                {
                    m_processError = counterclockwiseError;
                }
                else
                {
                    m_processError = clockwiseError;
                }
            }
            else
            {
                m_processVariable = 0.0;
                m_processError = 0.0;
            }
            break;
        case GraphSelection::kDrivePosition:
            // In meters.  Test routines set initial position to zero and introduce
            // step errors of 1_m, so already scaled, based on this maximum step error.
            m_processVariable = GetDriveDistance().to<double>();
            if (m_distanceVelocityNot)
            {
                m_processError = m_processVariable - m_commandedDistance.to<double>();
            }
            else
            {
                m_processError = 0.0;
            }
            break;
        case GraphSelection::kDriveVelocity:
            // In meters/s.  Scaled to maximum.
            m_processVariable = GetDriveVelocity().to<double>() / pidf::kDriveVelocityMaxVelocity;
            if (m_distanceVelocityNot)
            {
                m_processError = 0.0;
            }
            else
            {
                m_processError = m_processVariable - m_commandedVelocity.to<double>() / pidf::kDriveVelocityMaxVelocity;
            }
            break;
        case GraphSelection::kNone:
            break;
        }

        if (deltaTime > 0.0)
        {
            deltaProcessError -= m_processError;
            m_processFirstDerivative = deltaProcessError / deltaTime;
            deltaFirstDerivative -= m_processFirstDerivative;
            m_processSecondDerivitive = deltaFirstDerivative / deltaTime;
        }

        // Scale velocity and acceleration based on maximums.  XXX?
        switch (m_graphSelection)
        {
        case GraphSelection::kTurningRotation:
            m_processFirstDerivative /= pidf::kTurningPositionMaxVelocity.to<double>() / 180.0;
            m_processSecondDerivitive /= pidf::kTurningPositionMaxAcceleration.to<double>() / 180.0;
            break;
        case GraphSelection::kDrivePosition:
            m_processFirstDerivative /= pidf::kDrivePositionMaxVelocity;
            m_processSecondDerivitive /= pidf::kDrivePositionMaxAcceleration;
            break;
        case GraphSelection::kDriveVelocity:
            m_processFirstDerivative /= pidf::kDriveVelocityMaxAcceleration;
            m_processSecondDerivitive /= pidf::kDriveVelocityMaxJerk;
            break;
        case GraphSelection::kNone:
            break;
        }
    }
}

void SwerveModule::TurningPositionPID(double P, double I, double IZ, double IM, double D, double DF, double F, double V, double A) noexcept
{
    m_rioPIDController->SetPID(P, I, D);
    m_rioPIDController->SetConstraints(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
        units::angular_velocity::degrees_per_second_t{V},
        units::angular_acceleration::degrees_per_second_squared_t{A}});
    m_rioPID_F = F;

    m_turningPosition_P = P;
    m_turningPosition_I = I;
    m_turningPosition_IZ = IZ;
    m_turningPosition_IM = IM;
    m_turningPosition_D = D;
    m_turningPosition_DF = DF;
    m_turningPosition_F = F;
    m_turningPosition_V = V;
    m_turningPosition_A = A;

    SetTurningPositionPID();
}

void SwerveModule::DrivePositionPID(double P, double I, double IZ, double IM, double D, double DF, double F, double V, double A) noexcept
{
    m_drivePosition_P = P;
    m_drivePosition_I = I;
    m_drivePosition_IZ = IZ;
    m_drivePosition_IM = IM;
    m_drivePosition_D = D;
    m_drivePosition_DF = DF;
    m_drivePosition_F = F;
    m_drivePosition_V = V;
    m_drivePosition_A = A;

    SetDrivePositionPID();
}

void SwerveModule::DriveVelocityPID(double P, double I, double IZ, double IM, double D, double DF, double F, double V, double A) noexcept
{
    m_driveVelocity_P = P;
    m_driveVelocity_I = I;
    m_driveVelocity_IZ = IZ;
    m_driveVelocity_IM = IM;
    m_driveVelocity_D = D;
    m_driveVelocity_DF = DF;
    m_driveVelocity_F = F;
    m_driveVelocity_V = V;
    m_driveVelocity_A = A;

    SetDriveVelocityPID();
}

void SwerveModule::SetTurningPositionPID() noexcept
{
    // This isn't needed, but would be if turning PID is on SPARK MAX
    m_turningMotor->AddConfig(SmartMotorBase::ConfigMap{
        {"kP_0", double{m_turningPosition_P}},
        {"kI_0", double{m_turningPosition_I}},
        {"kD_0", double{m_turningPosition_D}},
        {"kF_0", double{m_turningPosition_F}},
        {"kIZone_0", double{m_turningPosition_IZ}},
        {"kIMaxAccum_0", double{m_turningPosition_IM}},
        {"kDFilter_0", double{m_turningPosition_DF}},
        {"kSmartMotionMaxVelocity_0", double{m_turningPosition_V}},
        {"kSmartMotionMaxAccel_0", double{m_turningPosition_A}},
        {"kSmartMotionAccelStrategy_0", uint{0}},
    });

    m_turningMotor->ApplyConfig(false);
}

void SwerveModule::SetDrivePositionPID() noexcept
{
    m_driveMotor->AddConfig(SmartMotorBase::ConfigMap{
        {"kP_0", double{m_drivePosition_P}},
        {"kI_0", double{m_drivePosition_I}},
        {"kD_0", double{m_drivePosition_D}},
        {"kF_0", double{m_drivePosition_F}},
        {"kIZone_0", double{m_drivePosition_IZ}},
        {"kIMaxAccum_0", double{m_drivePosition_IM}},
        {"kDFilter_0", double{m_drivePosition_DF}},
        {"kSmartMotionMaxVelocity_0", double{m_drivePosition_V}},
        {"kSmartMotionMaxAccel_0", double{m_drivePosition_A}},
        {"kSmartMotionAccelStrategy_0", uint{0}},
    });

    m_driveMotor->ApplyConfig(false);
}

void SwerveModule::SetDriveVelocityPID() noexcept
{
    m_driveMotor->AddConfig(SmartMotorBase::ConfigMap{
        {"kP_1", double{m_driveVelocity_P}},
        {"kI_1", double{m_driveVelocity_I}},
        {"kD_1", double{m_driveVelocity_D}},
        {"kF_1", double{m_driveVelocity_F}},
        {"kIZone_1", double{m_driveVelocity_IZ}},
        {"kIMaxAccum_1", double{m_driveVelocity_IM}},
        {"kDFilter_1", double{m_driveVelocity_DF}},
        {"kSmartMotionMaxVelocity_1", double{m_driveVelocity_V}},
        {"kSmartMotionMaxAccel_1", double{m_driveVelocity_A}},
        {"kSmartMotionAccelStrategy_1", uint{1}},
    });

    m_driveMotor->ApplyConfig(false);
}

void SwerveModule::SetStatusFramePeriods(GraphSelection graphSelection) noexcept
{
    const SmartMotorBase::ConfigMap slow{
        {"kStatus1", uint{250}}, // ms
        {"kStatus2", uint{250}}, // ms
    };

    const SmartMotorBase::ConfigMap fast{
        {"kStatus1", uint{10}}, // ms
        {"kStatus2", uint{10}}, // ms
    };

    switch (m_graphSelection)
    {
    case GraphSelection::kTurningRotation:
        m_turningMotor->AddConfig(fast);
        m_driveMotor->AddConfig(slow);
        break;
    case GraphSelection::kDrivePosition:
        m_turningMotor->AddConfig(slow);
        m_driveMotor->AddConfig(fast);
        break;
    case GraphSelection::kDriveVelocity:
        m_turningMotor->AddConfig(slow);
        m_driveMotor->AddConfig(fast);
        break;
    case GraphSelection::kNone:
        m_turningMotor->AddConfig(slow);
        m_driveMotor->AddConfig(slow);
        break;
    }

    m_turningMotor->ApplyConfig(false);
    m_driveMotor->ApplyConfig(false);
}

void SwerveModule::BurnConfig() noexcept
{
    m_turningMotor->ApplyConfig(true);
    m_driveMotor->ApplyConfig(true);
}

void SwerveModule::ClearFaults() noexcept
{
    m_turningMotor->ClearFaults();
    m_driveMotor->ClearFaults();
}
