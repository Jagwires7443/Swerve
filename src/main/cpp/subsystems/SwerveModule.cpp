// See https://github.com/wpilibsuite/allwpilib/blob/main/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/subsystems/SwerveModule.cpp.
// See https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control.

// A REV Robotics issue prevents running closed-loop control on the turning
// motor controller: https://trello.com/c/bvnPPcZD/108-add-continuous-pid-capability.
// The `m_rio` flag controls where this control is done, but the code is here
// to implement this either on the roboRIO or on the SPARK MAX.

#include "subsystems/SwerveModule.h"

#include "Constants.h"

#include <frc/RobotController.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <networktables/NetworkTableValue.h>
#include <rev/REVLibError.h>
#include <rev/ControlType.h>
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

namespace
{
    std::string FirmwareInfo(const uint32_t firmwareVersion, const std::string &firmwareString) noexcept
    {
        const std::bitset<8> firmwareFieldBuildL((firmwareVersion & 0x000000ff) >> 0);
        const std::bitset<8> firmwareFieldBuildH((firmwareVersion & 0x0000ff00) >> 8);
        const std::bitset<8> firmwareFieldMinor((firmwareVersion & 0x00ff0000) >> 16);
        const std::bitset<8> firmwareFieldMajor((firmwareVersion & 0xff000000) >> 24);

        const std::bitset<16> firmwareFieldBuild((firmwareFieldBuildH.to_ulong() << 8) |
                                                 (firmwareFieldBuildL.to_ulong()));

        std::stringstream firmwareVersionHex;

        firmwareVersionHex << std::internal << std::setfill('0') << std::setw(10)
                           << std::hex << std::showbase << firmwareVersion;

        std::string firmwareInfo;

        firmwareInfo += " FW(";
        firmwareInfo += firmwareVersionHex.str();
        firmwareInfo += "/";
        firmwareInfo += std::to_string(firmwareFieldMajor.to_ulong());
        firmwareInfo += ".";
        firmwareInfo += std::to_string(firmwareFieldMinor.to_ulong());
        firmwareInfo += ".";
        firmwareInfo += std::to_string(firmwareFieldBuild.to_ulong());
        firmwareInfo += "/\"";
        firmwareInfo += firmwareString;
        firmwareInfo += "\")";

        return firmwareInfo;
    }

    std::string FaultInfo(const uint16_t faults)
    {
        const std::bitset<16> faultBits(faults);

        // This works fine, but isn't very readable on the display.
        // return faultBits.to_string('.', '*');

        // SPARK MAX Fault Codes with shorthand characters used here.
        // B kBrownout
        // C kOvercurrent
        // W kIWDTReset
        // M kMotorFault
        // S kSensorFault
        // L kStall
        // E kEEPROMCRC
        // > kCANTX
        // < kCANRX
        // R kHasReset
        // D kDRVFault
        // O kOtherFault
        // ) kSoftLimitFwd
        // ( kSoftLimitRev
        // ] kHardLimitFwd
        // [ kHardLimitRev
        const char letters[] = {'B', 'C', 'W', 'M', 'S', 'L', 'E', '>', '<', 'R', 'D', 'O', ')', '(', ']', '['};

        std::string faultInfo;

        for (std::size_t i = faultBits.size(); i > 0; --i)
        {
            faultInfo += std::string(1, faultBits[i - 1] ? letters[i - 1] : '.');
        }

        return faultInfo;
    }
}

SwerveModule::SwerveModule(
    const char *const name,
    const int driveMotorCanID,
    const int turningMotorCanID,
    const int turningEncoderPort,
    const int alignmentOffset) noexcept : m_name{name},
                                          m_driveMotorCanID{driveMotorCanID},
                                          m_turningMotorCanID{turningMotorCanID},
                                          m_alignmentOffset{alignmentOffset}
{
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

    m_rioPIDController->EnableContinuousInput(-180_deg, +180_deg);

    // Construct turning absolute duty cycle encoder.  A `DigitalSource` is
    // required by the `DutyCycle` ctor.  Nothing here is expected to fail.
    // Note this has no explicit SetInverted() method; flip the turning motor
    // and it's incremental encoder in order to make things work out/line up.
    m_turningPositionSource = std::make_unique<frc::DigitalInput>(turningEncoderPort);
    m_turningPositionPWM = std::make_unique<frc::DutyCycle>(*m_turningPositionSource);

    // Motor controller configurations are only checked (or saved) in test mode
    // but a minimal amount is set up in these methods.
    ConstructTurningMotor();
    ConstructDriveMotor();

    ResetEncoders();

    std::printf(" OK.\n");
}

// Calulate absolute turning position in the range [-2048, +2048), from the raw
// absolute PWM encoder data.  No value is returned in the event the absolute
// position sensor itself is not returning valid data.  The alignment offset is
// optionally used to establish the zero position.

// This is a low-level routine, meant to be used only by the version of
// GetAbsolutePosition() with no arguments (below) or, from test mode.  It does
// not use standardized units and leaks knowledge of the sensor output range.
std::optional<int> SwerveModule::GetAbsolutePosition(const int frequency, const double output, const bool applyOffset) noexcept
{
    // SRX MAG ENCODER chip seems to be AS5045B; from the datasheet, position
    // is given by:
    //   ((t_on * 4098) / (t_on + t_off)) - 1;
    //   if this result is 4096, set it to 4095.
    // This computation results in a position in the range [0, 4096).

    // If the frequency isn't within the range specified in the data sheet,
    // return an error.  This range is [220, 268], with a nominal value of 244.
    // A tolerance of 12 (~5% of nominal) is provided for any measurment error.
    m_absoluteSensorGood = frequency >= 208 && frequency <= 280;

    if (!m_absoluteSensorGood)
    {
        return std::nullopt;
    }

    // GetOutput() is (t_on / (t_on + t_off)); this is all that we have; note
    // that this is sampled at a high frequency and the sampling window may not
    // be perfectly aligned with the signal, although the frequency should be
    // employed to ensure the sampling window is sized to a multiple of the
    // period of the signal.  So, t_on and t_off have a different scale in this
    // context, which is OK.  We are using the duty cycle (ratio) only.

    // Multiply by 4098 and subtract 1; should yield 0 - 4094, and also 4096.
    // Conditionals cover special case of 4096 and enforce the specified range.
    int position = std::lround(output * 4098.0) - 1;
    if (position < 0)
    {
        position = 0;
    }
    else if (position > 4095)
    {
        position = 4095;
    }

    // Now, shift the range from [0, 4096) to [-2048, +2048).
    position -= 2048;

    if (!applyOffset)
    {
        return position;
    }

    // There is a mechanical offset reflecting the alignment of the magnet with
    // repect to the axis of rotation of the wheel (and also any variance in
    // the alignment of the sensor).  Add this in here to reference position to
    // the desired zero position.
    position += m_alignmentOffset;
    if (position > 2047)
    {
        position -= 4096;
    }
    if (position < -2048)
    {
        position += 4096;
    }

    return position;
}

// As above; obtain raw data from DutyCycle object.
std::optional<units::angle::degree_t> SwerveModule::GetAbsolutePosition() noexcept
{
    const auto position = GetAbsolutePosition(m_turningPositionPWM->GetFrequency(),
                                              m_turningPositionPWM->GetOutput(),
                                              true);

    if (!position.has_value())
    {
        return std::nullopt;
    }

    m_turningPosition = units::angle::turn_t{static_cast<double>(position.value()) / 4096.0};

    return m_turningPosition;
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

void SwerveModule::DoSafeTurningMotor(const char *const what, std::function<void()> work) noexcept
{
    try
    {
        work();
    }
    catch (const std::exception &e)
    {
        m_turningPID = nullptr;
        m_turningEncoder = nullptr;
        m_turningMotor = nullptr;

        std::printf("Turning Motor %s %s exception: %s.\n", m_name.c_str(), what, e.what());
    }
    catch (...)
    {
        m_turningPID = nullptr;
        m_turningEncoder = nullptr;
        m_turningMotor = nullptr;

        std::printf("Turning Motor %s %s unknown exception.\n", m_name.c_str(), what);
    }
}

bool SwerveModule::DidSafeTurningMotor(const char *const what, std::function<bool()> work) noexcept
{
    bool result{false};

    try
    {
        result = work();
    }
    catch (const std::exception &e)
    {
        result = false;

        m_turningPID = nullptr;
        m_turningEncoder = nullptr;
        m_turningMotor = nullptr;

        std::printf("Turning Motor %s %s exception: %s.\n", m_name.c_str(), what, e.what());
    }
    catch (...)
    {
        result = false;

        m_turningPID = nullptr;
        m_turningEncoder = nullptr;
        m_turningMotor = nullptr;

        std::printf("Turning Motor %s %s unknown exception.\n", m_name.c_str(), what);
    }

    return result;
}

void SwerveModule::DoSafeDriveMotor(const char *const what, std::function<void()> work) noexcept
{
    try
    {
        work();
    }
    catch (const std::exception &e)
    {
        m_drivePID = nullptr;
        m_driveEncoder = nullptr;
        m_driveMotor = nullptr;

        std::printf("Drive Motor %s %s exception: %s.\n", m_name.c_str(), what, e.what());
    }
    catch (...)
    {
        m_drivePID = nullptr;
        m_driveEncoder = nullptr;
        m_driveMotor = nullptr;

        std::printf("Drive Motor %s %s unknown exception.\n", m_name.c_str(), what);
    }
}

bool SwerveModule::DidSafeDriveMotor(const char *const what, std::function<bool()> work) noexcept
{
    bool result{false};

    try
    {
        result = work();
    }
    catch (const std::exception &e)
    {
        result = false;

        m_drivePID = nullptr;
        m_driveEncoder = nullptr;
        m_driveMotor = nullptr;

        std::printf("Drive Motor %s %s exception: %s.\n", m_name.c_str(), what, e.what());
    }
    catch (...)
    {
        result = false;

        m_drivePID = nullptr;
        m_driveEncoder = nullptr;
        m_driveMotor = nullptr;

        std::printf("Drive Motor %s %s unknown exception.\n", m_name.c_str(), what);
    }

    return result;
}

// Construct and get/move turning motor objects.  It is possible for CAN or
// power issues to induce errors (particuarly from ctors), so do try/catch.
// The configuration is handled elsewhere; the expected case is that this has
// been saved, so that the motor controller comes up properly configured.

// This handles only those settings that are initially configured manually:
// kCanID (this cannot be configured programmatically), kMotorType/kSensorType,
// and kDataPortConfig; kIdleMode is not handled here).  To ensure improper
// inversions are never exposed, kAltEncoderInverted is handled here instead.
void SwerveModule::ConstructTurningMotor() noexcept
{
    m_turningPID = nullptr;
    m_turningEncoder = nullptr;
    m_turningMotor = nullptr;

    DoSafeTurningMotor("ctor", [&]() -> void
                       {
        m_turningMotor = std::make_unique<rev::CANSparkMax>(
            m_turningMotorCanID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        m_turningEncoder = std::make_unique<rev::SparkMaxAlternateEncoder>(
            m_turningMotor->GetAlternateEncoder(4096));
        m_turningPID = std::make_unique<rev::SparkMaxPIDController>(m_turningMotor->GetPIDController());

        if (!m_turningMotor)
        {
            throw std::runtime_error("m_turningMotor");
        }

        // Does not get sent to the motor controller, done locally.
        m_turningMotor->SetInverted(m_turningMotorInverted);

        if (m_turningMotor->ClearFaults() != rev::REVLibError::kOk)
        {
            throw std::runtime_error("ClearFaults()");
        }

        if (!m_turningEncoder || !m_turningPID)
        {
            throw std::runtime_error("m_turningEncoder");
        }

        if (m_turningEncoder->SetInverted(m_turningEncoderInverted) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetInverted()");
        }

        if (m_turningPID->SetFeedbackDevice(*m_turningEncoder) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetFeedbackDevice()");
        } });
}

// Construct and get/move drive motor objects.  It is possible for CAN or
// power issues to induce errors (particuarly from ctors), so do try/catch.
// The configuration is handled elsewhere; the expected case is that this has
// been saved, so that the motor controller comes up properly configured.

// This handles only those settings that are initially configured manually:
// kCanID (this cannot be configured programmatically), kMotorType/kSensorType,
// and kDataPortConfig; kIdleMode is not handled here).  Since coast/brake is
// sometimes flipped, ensure things start out in coast.
void SwerveModule::ConstructDriveMotor() noexcept
{
    m_drivePID = nullptr;
    m_driveEncoder = nullptr;
    m_driveMotor = nullptr;

    DoSafeDriveMotor("ctor", [&]() -> void
                     {
        m_driveMotor = std::make_unique<rev::CANSparkMax>(
            m_driveMotorCanID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        m_driveEncoder = std::make_unique<rev::SparkMaxRelativeEncoder>(
            m_driveMotor->GetEncoder());
        m_drivePID = std::make_unique<rev::SparkMaxPIDController>(m_driveMotor->GetPIDController());

        if (!m_driveMotor)
        {
            throw std::runtime_error("m_driveMotor");
        }

        // Does not get sent to the motor controller, done locally.
        m_driveMotor->SetInverted(m_driveMotorInverted);

        if (m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIdleMode()");
        }

        if (m_driveMotor->ClearFaults() != rev::REVLibError::kOk)
        {
            throw std::runtime_error("ClearFaults()");
        }

        if (!m_driveEncoder || !m_drivePID)
        {
            throw std::runtime_error("m_driveEncoder");
        }

        if (m_drivePID->SetFeedbackDevice(*m_driveEncoder) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetFeedbackDevice()");
        } });
}

bool SwerveModule::GetStatus() const noexcept
{
    return m_absoluteSensorGood &&
           m_turningMotor && m_turningMotorControllerValidated &&
           m_driveMotor && m_driveMotorControllerValidated;
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
    if (calculated > 0)
    {
        calculated += m_rioPID_F;
    }
    else if (calculated < 0)
    {
        calculated -= m_rioPID_F;
    }

    DoSafeTurningMotor("Periodic()", [&]() -> void
                       {
        if (m_turningMotor)
        {
            // Use voltage compensation, to offset low battery voltage.
            m_turningMotor->SetVoltage(calculated * 12_V);
        } });
}

void SwerveModule::ResetTurning() noexcept
{
    const auto position = GetAbsolutePosition();

    // If absolute position isn't available, there's no basis for resetting and
    // it's better to just let any old data stand.  This is when manual
    // pointing of the modules before a match could ensure a reasonable zero.
    // Of course, if anything is reset, etc. all bets are off -- still, leaving
    // things alone is the best that can be done under these circumstances.
    if (!position.has_value())
    {
        return;
    }

    DoSafeTurningMotor("ResetTurning()", [&]() -> void
                       {
        if (m_turningEncoder)
        {
            // Turning position is in rotations.  Any under/overflow should
            // be wrapped but, in any case, a reset will place things
            // within [-0.5, +0.5).
            if (m_turningEncoder->SetPosition(
                    units::angle::turn_t(position.value()).to<double>()) != rev::REVLibError::kOk)
            {
                throw std::runtime_error("SetPosition()");
            }
        } });
}

void SwerveModule::ResetDrive() noexcept
{
    DoSafeDriveMotor("ResetDrive()", [&]() -> void
                     {
        if (m_driveEncoder)
        {
            if (m_driveEncoder->SetPosition(0.0) != rev::REVLibError::kOk)
            {
                throw std::runtime_error("SetPosition()");
            }
        } });
}

units::angle::degree_t SwerveModule::GetTurningPosition() noexcept
{
    const auto position = GetAbsolutePosition();

    if (position.has_value())
    {
        return position.value();
    }

    // Absolute encoder is strongly preferred -- it has very low latency,
    // and is absolute.  Since the same encoder is used with the SPARK MAX,
    // the resolution and direct measurement of turning angle are identical
    // (so long as the incremental encoder was successfully reset at some
    // point and there has been no reset or similar problem).  But, this is
    // the best that can be done here.
    units::angle::degree_t encoderPosition{0};

    DoSafeTurningMotor("GetState()", [&]() -> void
                       {
        if (m_turningEncoder)
        {
            encoderPosition = units::angle::turn_t{m_turningEncoder->GetPosition()};
        } });

    while (encoderPosition < 180_deg)
    {
        encoderPosition += 360_deg;
    }
    while (encoderPosition >= 180_deg)
    {
        encoderPosition -= 360_deg;
    }

    m_turningPosition = encoderPosition;

    return encoderPosition;
}

void SwerveModule::SetTurningPosition(const units::angle::degree_t position) noexcept
{
    units::angle::degree_t adjustedPosition = position;

    // Defensive coding; could use fmod(adjustedPosition, 360_deg) here, except
    // this would still require some logic because of sign and should never run
    // in any case.
    while (adjustedPosition < -180_deg)
    {
        adjustedPosition += 360_deg;
    }
    while (adjustedPosition >= +180_deg)
    {
        adjustedPosition -= 360_deg;
    }

    m_commandedHeading = adjustedPosition;

    m_rioPIDController->SetGoal(adjustedPosition);

    if (m_rio || m_testModeControl || m_testModeTurningVoltage != 0.0)
    {
        return;
    }

    DoSafeTurningMotor("SetTurningPosition()", [&]() -> void
                       {
        if (m_turningPID)
        {
            if (m_turningPID->SetReference(
                    units::angle::turn_t(adjustedPosition).to<double>(), rev::CANSparkMax::ControlType::kPosition) != rev::REVLibError::kOk)
            {
                throw std::runtime_error("SetReference()");
            }
        } });
}

bool SwerveModule::CheckTurningPosition(const units::angle::degree_t tolerance) noexcept
{
    units::angle::degree_t error = GetTurningPosition() - m_commandedHeading;

    if (error < -180_deg)
    {
        error += 360_deg;
    }
    else if (error >= 180_deg)
    {
        error -= 360_deg;
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
    units::length::meter_t result{0};

    DoSafeDriveMotor("GetDriveDistance()", [&]() -> void
                     {
        if (m_driveEncoder)
        {
            result = m_driveEncoder->GetPosition() * physical::kDriveMetersPerRotation;
        } });

    return result;
}

void SwerveModule::SetDriveDistance(units::length::meter_t distance) noexcept
{
    m_distanceVelocityNot = true;
    m_commandedDistance = distance;
    m_commandedVelocity = 0_mps;

    if (m_testModeControl || m_testModeDriveVoltage != 0.0)
    {
        return;
    }

    DoSafeDriveMotor("SetDriveDistance()", [&]() -> void
                     {
        if (m_driveMotor && m_drivePID)
        {
            if (m_turningPositionAsCommanded)
            {
                // SetReference(SetPoint, rev::ControlType::kSmartMotion, 0, FeedForward)
                // XXX Use m_drivePosition_F here?
                if (m_drivePID->SetReference(
                        (distance / physical::kDriveMetersPerRotation).to<double>(), rev::CANSparkMax::ControlType::kSmartMotion) != rev::REVLibError::kOk)
                {
                    throw std::runtime_error("SetReference()");
                }

                if (!m_brakeApplied)
                {
                    if (m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) != rev::REVLibError::kOk)
                    {
                        throw std::runtime_error("SetIdleMode()");
                    }

                    m_brakeApplied = true;
                }
            }
            else
            {
                m_driveMotor->StopMotor();

                if (m_brakeApplied)
                {
                    if (m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk)
                    {
                        throw std::runtime_error("SetIdleMode()");
                    }

                    m_brakeApplied = false;
                }
            }
        } });
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
    units::velocity::meters_per_second_t result{0};

    DoSafeDriveMotor("GetDriveVelocity()", [&]() -> void
                     {
        if (m_driveEncoder)
        {
            // GetVelocity() returns revolutions per minute, need meters per second
            result = m_driveEncoder->GetVelocity() * physical::kDriveMetersPerRotation / 60_s;
        } });

    return result;
}

void SwerveModule::SetDriveVelocity(units::velocity::meters_per_second_t velocity) noexcept
{
    m_distanceVelocityNot = false;
    m_commandedDistance = 0_m;
    m_commandedVelocity = velocity;

    if (m_testModeControl || m_testModeDriveVoltage != 0.0)
    {
        return;
    }

    DoSafeDriveMotor("SetDriveVelocity()", [&]() -> void
                     {
        if (m_driveMotor && m_drivePID)
        {
            if (m_turningPositionAsCommanded)
            {
#if 0
                // SetReference(SetPoint, rev::ControlType::kSmartVelocity, 1, FeedForward)
                if (m_drivePID->SetReference(
                        (velocity * 60_s / physical::kDriveMetersPerRotation).to<double>(), rev::CANSparkMax::ControlType::kSmartVelocity, 1) != rev::REVLibError::kOk)
                {
                    throw std::runtime_error("SetReference()");
                }
#else
                m_driveMotor->SetVoltage(velocity * 1_s / physical::kDriveMetersPerRotation * 1_V);
#endif

                if (!m_brakeApplied && m_commandedBrake)
                {
                    if (m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) != rev::REVLibError::kOk)
                    {
                        throw std::runtime_error("SetIdleMode()");
                    }

                    m_brakeApplied = true;
                }
            }
            else
            {
                m_driveMotor->StopMotor();

                if (m_brakeApplied)
                {
                    if (m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk)
                    {
                        throw std::runtime_error("SetIdleMode()");
                    }

                    m_brakeApplied = false;
                }
            }
        } });
}

const frc::SwerveModuleState SwerveModule::GetState() noexcept
{
    frc::SwerveModuleState result;

    result.angle = frc::Rotation2d(GetTurningPosition());
    result.speed = GetDriveVelocity();

    return result;
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState &referenceState) noexcept
{
    // Optimize the reference state, to avoid spinning further than 90 degrees.
    frc::SwerveModuleState state = referenceState;

    const auto position = GetAbsolutePosition();

    if (position.has_value())
    {
        state = frc::SwerveModuleState::Optimize(referenceState, frc::Rotation2d(position.value()));
    }

    SetTurningPosition(state.angle.Degrees());

    SetDriveVelocity(state.speed);
}

void SwerveModule::ResetEncoders() noexcept
{
    ResetTurning();
    ResetDrive();
}

// This sets up the test mode shuffleboard tab; everything is specified
// programmatically.
void SwerveModule::TestInit() noexcept
{
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
            .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(3.0))});

    frc::ShuffleboardLayout &shuffleboardLayoutTurningMotor =
        shuffleboardTab.GetLayout("Turning Motor",
                                  frc::BuiltInLayouts::kGrid)
            .WithPosition(8, 0)
            .WithSize(20, 6)
            .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(6.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

    frc::ShuffleboardLayout &shuffleboardLayoutDriveMotor =
        shuffleboardTab.GetLayout("Drive Motor",
                                  frc::BuiltInLayouts::kGrid)
            .WithPosition(8, 7)
            .WithSize(20, 6)
            .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(6.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

    // Setup widgets.

    m_turningPositionStatus = &shuffleboardLayoutTurningPosition.Add("Status", false)
                                   .WithPosition(0, 2)
                                   .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    m_turningPositionFrequency = &shuffleboardLayoutTurningPosition.Add("Frequency", 0)
                                      .WithPosition(0, 0);
    m_turningPositionOutput = &shuffleboardLayoutTurningPosition.Add("Output", 0.0)
                                   .WithPosition(2, 0);
    m_turningPositionCommandDiscrepancy = &shuffleboardLayoutTurningPosition.Add("PID Error", 0)
                                               .WithPosition(0, 1);
    m_turningPositionCommanded = &shuffleboardLayoutTurningPosition.Add("Commanded", 0)
                                      .WithPosition(1, 1);
    m_turningPositionEncoderDiscrepancy = &shuffleboardLayoutTurningPosition.Add("Motor Discrepancy", 0)
                                               .WithPosition(2, 1);
    m_turningPositionAlignment = &shuffleboardLayoutTurningPosition.Add("Alignment", 0)
                                      .WithPosition(2, 2);
    m_turningPositionPosition = &shuffleboardLayoutTurningPosition.Add("Position", 0)
                                     .WithPosition(1, 2);
    m_turningPositionHeading = &shuffleboardLayoutTurningPosition.Add("Heading", m_headingGyro)
                                    .WithPosition(1, 0)
                                    .WithWidget(frc::BuiltInWidgets::kGyro)
                                    .WithProperties(wpi::StringMap<std::shared_ptr<nt::Value>>{
                                        std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});

    m_turningMotorStatus = &shuffleboardLayoutTurningMotor.Add("Status", false)
                                .WithPosition(0, 1)
                                .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    m_turningMotorTemperature = &shuffleboardLayoutTurningMotor.Add("Temperature", 0.0)
                                     .WithPosition(0, 0);
    m_turningMotorFaults = &shuffleboardLayoutTurningMotor.Add("Faults", "")
                                .WithPosition(1, 0);
    m_turningMotorStickyFaults = &shuffleboardLayoutTurningMotor.Add("StickyFaults", "")
                                      .WithPosition(1, 1);
    m_turningMotorVoltage = &shuffleboardLayoutTurningMotor.Add("Voltage", 0.0)
                                 .WithPosition(2, 0);
    m_turningMotorCurrent = &shuffleboardLayoutTurningMotor.Add("Current", 0.0)
                                 .WithPosition(2, 1);
    m_turningMotorSpeed = &shuffleboardLayoutTurningMotor.Add("Speed (Commanded)", 0.0)
                               .WithPosition(3, 1)
                               .WithWidget(frc::BuiltInWidgets::kNumberBar);
    m_turningMotorPercent = &shuffleboardLayoutTurningMotor.Add("Percent (Actual)", 0.0)
                                 .WithPosition(3, 0)
                                 .WithWidget(frc::BuiltInWidgets::kNumberBar);
    m_turningMotorDistance = &shuffleboardLayoutTurningMotor.Add("Distance (Rotations)", 0.0)
                                  .WithPosition(4, 0);
    m_turningMotorVelocity = &shuffleboardLayoutTurningMotor.Add("Velocity (Rot per sec)", 0.0)
                                  .WithPosition(4, 1);
    m_turningMotorControl = &shuffleboardLayoutTurningMotor.Add("Control", 0.0)
                                 .WithPosition(5, 0)
                                 .WithWidget(frc::BuiltInWidgets::kNumberSlider);
    m_turningMotorReset = &shuffleboardLayoutTurningMotor.Add("Reset", false)
                               .WithPosition(5, 1)
                               .WithWidget(frc::BuiltInWidgets::kToggleButton);

    m_driveMotorStatus = &shuffleboardLayoutDriveMotor.Add("Status", false)
                              .WithPosition(0, 1)
                              .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    m_driveMotorTemperature = &shuffleboardLayoutDriveMotor.Add("Temperature", 0.0)
                                   .WithPosition(0, 0);
    m_driveMotorFaults = &shuffleboardLayoutDriveMotor.Add("Faults", "")
                              .WithPosition(1, 0);
    m_driveMotorStickyFaults = &shuffleboardLayoutDriveMotor.Add("StickyFaults", "")
                                    .WithPosition(1, 1);
    m_driveMotorVoltage = &shuffleboardLayoutDriveMotor.Add("Voltage", 0.0)
                               .WithPosition(2, 0);
    m_driveMotorCurrent = &shuffleboardLayoutDriveMotor.Add("Current", 0.0)
                               .WithPosition(2, 1);
    m_driveMotorSpeed = &shuffleboardLayoutDriveMotor.Add("Speed (Commanded)", 0.0)
                             .WithPosition(3, 1)
                             .WithWidget(frc::BuiltInWidgets::kNumberBar);
    m_driveMotorPercent = &shuffleboardLayoutDriveMotor.Add("Percent (Actual)", 0.0)
                               .WithPosition(3, 0)
                               .WithWidget(frc::BuiltInWidgets::kNumberBar);
    m_driveMotorDistance = &shuffleboardLayoutDriveMotor.Add("Distance (Rotations)", 0.0)
                                .WithPosition(4, 0);
    m_driveMotorVelocity = &shuffleboardLayoutDriveMotor.Add("Velocity (Rot per sec)", 0.0)
                                .WithPosition(4, 1);
    m_driveMotorControl = &shuffleboardLayoutDriveMotor.Add("Control", 0.0)
                               .WithPosition(5, 0)
                               .WithWidget(frc::BuiltInWidgets::kNumberSlider);
    m_driveMotorReset = &shuffleboardLayoutDriveMotor.Add("Reset", false)
                             .WithPosition(5, 1)
                             .WithWidget(frc::BuiltInWidgets::kToggleButton);

    std::printf(" OK.\n");
}

void SwerveModule::TestExit() noexcept
{
    m_testModeControl = false;
    m_turningMotorControllerValidated = true;
    m_driveMotorControllerValidated = true;

    // Robot is likely not enabled at this stage, but go ahead and command turn
    // to home position.  This will take effect when enabled.
    SetTurningPosition(0_deg);
}

// This updates data in the Shuffleboard tab and handles HMI interaction.
void SwerveModule::TestPeriodic() noexcept
{
    // Read controls information from Shuffleboard and manage interactive UI.
    bool zeroTurning = m_turningMotorReset->GetEntry().GetBoolean(false);
    double setTurning = m_turningMotorControl->GetEntry().GetDouble(0.0);
    bool zeroDrive = m_driveMotorReset->GetEntry().GetBoolean(false);
    double setDrive = m_driveMotorControl->GetEntry().GetDouble(0.0);

    if (zeroTurning)
    {
        m_turningMotorReset->GetEntry().SetBoolean(false);
        m_turningMotorControl->GetEntry().SetDouble(0.0);
        setTurning = 0.0;
    }

    if (zeroDrive)
    {
        m_driveMotorReset->GetEntry().SetBoolean(false);
        m_driveMotorControl->GetEntry().SetDouble(0.0);
        setDrive = 0.0;
    }

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

        m_verifyMotorControllersWhen = now + 10s;

        m_turningMotorControllerValidated = VerifyTurningMotorControllerConfig();
        m_driveMotorControllerValidated = VerifyDriveMotorControllerConfig();
    }

    // Obtain raw data from DutyCycle object; absolute position without
    // adding in any current alignment offset, at least here.
    int frequency = m_turningPositionPWM->GetFrequency();
    double output = m_turningPositionPWM->GetOutput();
    auto position = GetAbsolutePosition(frequency, output, false);

    // This provides a rough means of zeroing the turning position.
    if (position.has_value())
    {
        if (zeroTurning)
        {
            // Work out new alignment so position becomes zero.
            m_alignmentOffset = -position.value();
            if (m_alignmentOffset == 2048)
            {
                m_alignmentOffset = -2048;
            }

            position = 0;
        }
        else
        {
            position = position.value() + m_alignmentOffset;
            if (position > 2047)
            {
                position = position.value() - 4096;
            }
            if (position < -2048)
            {
                position = position.value() + 4096;
            }
        }
    }

    // Temporarily store raw data from one of the SPARK MAX motor controllers.
    double temperature;
    uint16_t faults;
    uint16_t stickyFaults;
    double voltage;
    double current;
    double speed;
    double percent;
    double distance;
    double velocity;

    temperature = 0.0;
    faults = 0;
    stickyFaults = 0;
    voltage = 0.0;
    current = 0.0;
    speed = 0.0;
    percent = 0.0;
    distance = 0.0;
    velocity = 0.0;

    // Obtain raw data from turning CANSparkMax and set the output.
    DoSafeTurningMotor("TestPeriodic()", [&]() -> void
                       {
        if (m_turningMotor)
        {
            temperature = m_turningMotor->GetMotorTemperature();
            faults = m_turningMotor->GetFaults();
            stickyFaults = m_turningMotor->GetStickyFaults();
            voltage = m_turningMotor->GetBusVoltage();
            current = m_turningMotor->GetOutputCurrent();
            speed = m_turningMotor->Get();                // Commanded setting [-1, 1]
            percent = m_turningMotor->GetAppliedOutput(); // Actual setting [-1, 1]
        }

        if (m_turningEncoder)
        {
            distance = m_turningEncoder->GetPosition();        // Rotations
            velocity = m_turningEncoder->GetVelocity() / 60.0; // Rotations per second

            if (zeroTurning)
            {
                // Logic above ensures that `position` is now zero; reset the
                // turning motor controller encoder to reflect this.
                if (m_turningEncoder->SetPosition(0.0) != rev::REVLibError::kOk)
                {
                    throw std::runtime_error("SetPosition()");
                }
            }
        }

        if (m_turningMotor)
        {
            if (zeroTurning)
            {
                if (m_turningMotor->ClearFaults() != rev::REVLibError::kOk)
                {
                    throw std::runtime_error("ClearFaults()");
                }
            }

            if (m_testModeControl)
            {
                m_turningMotor->Set(setTurning);
            }
            else if (m_testModeTurningVoltage != 0.0)
            {
                m_turningMotor->SetVoltage(m_testModeTurningVoltage * 1_V);
            }
        } });

    // Compute the discrepancy between the absolute encoder and the incremental
    // encoder (as seen by the motor controller).  This should be very small if
    // the module is not turning as the data is being collected.  However, if
    // the module is turning, some discrepany is expected.  For some of these
    // parameters, it may be useful to graph values over time, as the module is
    // rotated at constant angular velocity.  This would make it easy to find
    // the min/max or to spot any discontinuities, for example.  Also, computes
    // the error in achieving the commanded turning position (useful when
    // operating in closed-loop).
    const double commandedHeading = m_commandedHeading.to<double>();
    double actualHeading{0.0};
    double error{0.0};
    int discrepancy{0};

    if (position.has_value())
    {
        actualHeading = static_cast<double>(position.value()) * 360.0 / 4096.0;
        error = actualHeading - commandedHeading;

        if (error < -180.0)
        {
            error += 360.0;
        }
        else if (error >= 180.0)
        {
            error -= 360.0;
        }

        units::angle::turn_t encoderPosition{distance};

        while (encoderPosition < -180_deg)
        {
            encoderPosition += 360_deg;
        }
        while (encoderPosition >= 180_deg)
        {
            encoderPosition -= 360_deg;
        }

        discrepancy = position.value() - std::lround(encoderPosition.to<double>() * 4096.0);

        if (discrepancy < -2048)
        {
            discrepancy += 4096;
        }
        else if (discrepancy > 2047)
        {
            discrepancy -= 4096;
        }
    }

    // Update shuffleboard data for DutyCycle object.
    m_turningPositionStatus->GetEntry().SetBoolean(position.has_value());

    // From here on, ensure there is some value for `position`.
    if (!position.has_value())
    {
        position = 0;
    }

    m_turningPositionFrequency->GetEntry().SetDouble(static_cast<double>(frequency));
    m_turningPositionOutput->GetEntry().SetDouble(output);
    m_turningPositionAlignment->GetEntry().SetDouble(static_cast<double>(m_alignmentOffset));
    m_turningPositionPosition->GetEntry().SetDouble(static_cast<double>(position.value()));
    m_headingGyro.Set(actualHeading);
    m_turningPositionCommanded->GetEntry().SetDouble(commandedHeading);
    m_turningPositionCommandDiscrepancy->GetEntry().SetDouble(error);
    m_turningPositionEncoderDiscrepancy->GetEntry().SetDouble(static_cast<double>(discrepancy));

    // Update shuffleboard data for turning CANSparkMax.
    m_turningMotorStatus->GetEntry().SetBoolean(m_turningMotor && m_turningMotorControllerValidated);
    m_turningMotorTemperature->GetEntry().SetDouble(temperature);
    m_turningMotorFaults->GetEntry().SetString(FaultInfo(faults));
    m_turningMotorStickyFaults->GetEntry().SetString(FaultInfo(stickyFaults));
    m_turningMotorVoltage->GetEntry().SetDouble(voltage);
    m_turningMotorCurrent->GetEntry().SetDouble(current);
    m_turningMotorSpeed->GetEntry().SetDouble(speed);
    m_turningMotorPercent->GetEntry().SetDouble(percent);
    m_turningMotorDistance->GetEntry().SetDouble(distance);
    m_turningMotorVelocity->GetEntry().SetDouble(velocity);

    temperature = 0.0;
    faults = 0;
    stickyFaults = 0;
    voltage = 0.0;
    current = 0.0;
    speed = 0.0;
    percent = 0.0;
    distance = 0.0;
    velocity = 0.0;

    // Obtain raw data from drive CANSparkMax and set the output.
    DoSafeDriveMotor("TestPeriodic()", [&]() -> void
                     {
        if (m_driveMotor)
        {
            temperature = m_driveMotor->GetMotorTemperature();
            faults = m_driveMotor->GetFaults();
            stickyFaults = m_driveMotor->GetStickyFaults();
            voltage = m_driveMotor->GetBusVoltage();
            current = m_driveMotor->GetOutputCurrent();
            speed = m_driveMotor->Get();                // Commanded setting [-1, 1]
            percent = m_driveMotor->GetAppliedOutput(); // Actual setting [-1, 1]
        }

        if (m_driveEncoder)
        {
            distance = m_driveEncoder->GetPosition();        // Rotations
            velocity = m_driveEncoder->GetVelocity() / 60.0; // Rotations per second

            if (zeroDrive)
            {
                if (m_driveEncoder->SetPosition(0.0) != rev::REVLibError::kOk)
                {
                    throw std::runtime_error("SetPosition()");
                }
            }
        }

        if (m_driveMotor)
        {
            if (zeroDrive)
            {
                if (m_driveMotor->ClearFaults() != rev::REVLibError::kOk)
                {
                    throw std::runtime_error("ClearFaults()");
                }
            }

            if (m_testModeControl)
            {
                m_driveMotor->Set(setDrive);
            }
            else if (m_testModeDriveVoltage != 0.0)
            {
                m_driveMotor->SetVoltage(m_testModeDriveVoltage * 1_V);
            }
        } });

    // Update shuffleboard data for drive CANSparkMax.
    m_driveMotorStatus->GetEntry().SetBoolean(m_driveMotor && m_driveMotorControllerValidated);
    m_driveMotorTemperature->GetEntry().SetDouble(temperature);
    m_driveMotorFaults->GetEntry().SetString(FaultInfo(faults));
    m_driveMotorStickyFaults->GetEntry().SetString(FaultInfo(stickyFaults));
    m_driveMotorVoltage->GetEntry().SetDouble(voltage);
    m_driveMotorCurrent->GetEntry().SetDouble(current);
    m_driveMotorSpeed->GetEntry().SetDouble(speed);
    m_driveMotorPercent->GetEntry().SetDouble(percent);
    m_driveMotorDistance->GetEntry().SetDouble(distance);
    m_driveMotorVelocity->GetEntry().SetDouble(velocity);

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
            // In degrees, [-180, +180); normalize to [-1.0, +1.0).
            m_processVariable = actualHeading / 180.0;
            m_processError = error / 180.0;
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
                m_processError = m_processVariable - m_commandedVelocity.to<double>();
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

        // Scale velocity and acceleration based on maximums.
        switch (m_graphSelection)
        {
        case GraphSelection::kTurningRotation:
            m_processFirstDerivative /= pidf::kTurningPositionMaxVelocity.to<double>();
            m_processSecondDerivitive /= pidf::kTurningPositionMaxAcceleration.to<double>();
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

    if (zeroTurning && !m_turningMotorControllerValidated)
    {
        CreateTurningMotorControllerConfig();
    }

    if (zeroDrive && !m_driveMotorControllerValidated)
    {
        CreateDriveMotorControllerConfig();
    }
}

void SwerveModule::TurningPositionPID(double P, double I, double IZ, double IM, double D, double DF, double F) noexcept
{
    m_rioPIDController->SetPID(P, I, D);
    m_rioPID_F = F;

    m_turningPosition_P = P;
    m_turningPosition_I = I;
    m_turningPosition_IZ = IZ;
    m_turningPosition_IM = IM;
    m_turningPosition_D = D;
    m_turningPosition_DF = DF;
    m_turningPosition_F = F;

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
    DoSafeTurningMotor("SetTurningPositionPID()", [&]() -> void
                       {
        if (!m_turningPID)
        {
            throw std::runtime_error("m_turningPID");
        }
        if (m_turningPID->SetP(m_turningPosition_P) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetP()");
        }
        if (m_turningPID->SetI(m_turningPosition_I) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetI()");
        }
        if (m_turningPID->SetIZone(m_turningPosition_IZ) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIZone()");
        }
        if (m_turningPID->SetIMaxAccum(m_turningPosition_IM) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIMaxAccum()");
        }
        if (m_turningPID->SetD(m_turningPosition_D) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetD()");
        }
        if (m_turningPID->SetDFilter(m_turningPosition_DF) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetDFilter()");
        }
        if (m_turningPID->SetFF(m_turningPosition_F) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetFF()");
        } });
}

void SwerveModule::SetDrivePositionPID() noexcept
{
    DoSafeDriveMotor("SetDrivePositionPID()", [&]() -> void
                     {
        if (!m_drivePID)
        {
            throw std::runtime_error("m_drivePID");
        }
        if (m_drivePID->SetP(m_drivePosition_P) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetP()");
        }
        if (m_drivePID->SetI(m_drivePosition_I) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetI()");
        }
        if (m_drivePID->SetIZone(m_drivePosition_IZ) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIZone()");
        }
        if (m_drivePID->SetIMaxAccum(m_drivePosition_IM) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIMaxAccum()");
        }
        if (m_drivePID->SetD(m_drivePosition_D) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetD()");
        }
        if (m_drivePID->SetDFilter(m_drivePosition_DF) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetDFilter()");
        }
        if (m_drivePID->SetFF(m_drivePosition_F) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetFF()");
        }
        if (m_drivePID->SetSmartMotionMaxVelocity(m_drivePosition_V) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetSmartMotionMaxVelocity()");
        }
        if (m_drivePID->SetSmartMotionMaxAccel(m_drivePosition_A) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetSmartMotionMaxAccel()");
        } });
}

void SwerveModule::SetDriveVelocityPID() noexcept
{
    DoSafeDriveMotor("SetDriveVelocityPID()", [&]() -> void
                     {
        if (!m_drivePID)
        {
            throw std::runtime_error("m_drivePID");
        }
        if (m_drivePID->SetP(m_driveVelocity_P, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetP()");
        }
        if (m_drivePID->SetI(m_driveVelocity_I, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetI()");
        }
        if (m_drivePID->SetIZone(m_driveVelocity_IZ, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIZone()");
        }
        if (m_drivePID->SetIMaxAccum(m_driveVelocity_IM, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIMaxAccum()");
        }
        if (m_drivePID->SetD(m_driveVelocity_D, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetD()");
        }
        if (m_drivePID->SetDFilter(m_driveVelocity_DF, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetDFilter()");
        }
        if (m_drivePID->SetFF(m_driveVelocity_F, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetFF()");
        }
        if (m_drivePID->SetSmartMotionMaxVelocity(m_driveVelocity_V, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetSmartMotionMaxVelocity()");
        }
        if (m_drivePID->SetSmartMotionMaxAccel(m_driveVelocity_A, 1) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetSmartMotionMaxAccel()");
        } });
}

bool SwerveModule::VerifyTurningMotorControllerConfig() noexcept
{
    if (!m_turningMotor)
    {
        return false;
    }

    return DidSafeTurningMotor("VerifyTurningMotorControllerConfig()", [&]() -> bool
                               {
        if (!m_turningMotor || !m_turningEncoder || !m_turningPID)
        {
            throw std::runtime_error("m_turningMotor");
        }

        std::string turningMotorControllerConfig;

        uint32_t firmwareVersion = m_turningMotor->GetFirmwareVersion();
        if (firmwareVersion != firmware::kSparkMaxFirmwareVersion)
        {
            turningMotorControllerConfig += FirmwareInfo(firmwareVersion, m_turningMotor->GetFirmwareString());
        }
        if (m_turningMotor->GetMotorType() != rev::CANSparkMaxLowLevel::MotorType::kBrushless)
        {
            turningMotorControllerConfig += " MT";
        }
        if (m_turningMotor->GetIdleMode() != rev::CANSparkMax::IdleMode::kBrake)
        {
            turningMotorControllerConfig += " IM";
        }
        if (m_turningMotor->GetInverted() != m_turningMotorInverted)
        {
            turningMotorControllerConfig += " MI";
        }

        if (m_turningEncoder->GetInverted() != m_turningEncoderInverted)
        {
            turningMotorControllerConfig += " EI";
        }

        if (m_turningPID->GetP() != m_turningPosition_P)
        {
            turningMotorControllerConfig += " Pp";
        }
        if (m_turningPID->GetI() != m_turningPosition_I)
        {
            turningMotorControllerConfig += " Ip";
        }
        if (m_turningPID->GetIZone() != m_turningPosition_IZ)
        {
            turningMotorControllerConfig += " IZp";
        }
        if (m_turningPID->GetIMaxAccum() != m_turningPosition_IM)
        {
            turningMotorControllerConfig += " IMp";
        }
        if (m_turningPID->GetD() != m_turningPosition_D)
        {
            turningMotorControllerConfig += " Dp";
        }
        if (m_turningPID->GetDFilter() != m_turningPosition_DF)
        {
            turningMotorControllerConfig += " DFp";
        }
        if (m_turningPID->GetFF() != m_turningPosition_F)
        {
            turningMotorControllerConfig += " Fp";
        }

        if (turningMotorControllerConfig.empty())
        {
            m_turningMotorControllerConfig.clear();

            return true;
        }

        if (turningMotorControllerConfig != m_turningMotorControllerConfig)
        {
            printf("%s Turning Motor Config:%s\n", m_name.c_str(), turningMotorControllerConfig.c_str());

            m_turningMotorControllerConfig.swap(turningMotorControllerConfig);
        }

        return false; });
}

bool SwerveModule::VerifyDriveMotorControllerConfig() noexcept
{
    if (!m_driveMotor)
    {
        return false;
    }

    return DidSafeDriveMotor("VerifyDriveMotorControllerConfig()", [&]() -> bool
                             {
        if (!m_driveMotor || !m_drivePID)
        {
            throw std::runtime_error("m_driveMotor");
        }

        std::string driveMotorControllerConfig;

        uint32_t firmwareVersion = m_driveMotor->GetFirmwareVersion();
        if (firmwareVersion != firmware::kSparkMaxFirmwareVersion)
        {
            driveMotorControllerConfig += FirmwareInfo(firmwareVersion, m_driveMotor->GetFirmwareString());
        }
        if (m_driveMotor->GetMotorType() != rev::CANSparkMaxLowLevel::MotorType::kBrushless)
        {
            driveMotorControllerConfig += " MT";
        }
        // Note that this setting may be flipped while running, but not when in
        // test mode.  In any case, do not fail if this is set as expected,
        // even if the current setting is not the same as that saved on the
        // motor controller.
        if (m_driveMotor->GetIdleMode() != (m_brakeApplied ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast))
        {
            driveMotorControllerConfig += " IM";
        }
        if (m_driveMotor->GetInverted() != m_driveMotorInverted)
        {
            driveMotorControllerConfig += " MI";
        }

        if (m_drivePID->GetP() != m_drivePosition_P)
        {
            driveMotorControllerConfig += " Pp";
        }
        if (m_drivePID->GetI() != m_drivePosition_I)
        {
            driveMotorControllerConfig += " Ip";
        }
        if (m_drivePID->GetIAccum() != m_drivePosition_IZ)
        {
            driveMotorControllerConfig += " IZp";
        }
        if (m_drivePID->GetIMaxAccum() != m_drivePosition_IM)
        {
            driveMotorControllerConfig += " IMp";
        }
        if (m_drivePID->GetD() != m_drivePosition_D)
        {
            driveMotorControllerConfig += " Dp";
        }
        if (m_drivePID->GetDFilter() != m_drivePosition_DF)
        {
            driveMotorControllerConfig += " DFp";
        }
        if (m_drivePID->GetFF() != m_drivePosition_F)
        {
            driveMotorControllerConfig += " Fp";
        }
        if (m_drivePID->GetSmartMotionMaxVelocity() != m_drivePosition_V)
        {
            driveMotorControllerConfig += " Vp";
        }
        if (m_drivePID->GetSmartMotionMaxAccel() != m_drivePosition_A)
        {
            driveMotorControllerConfig += " Ap";
        }

        if (m_drivePID->GetP(1) != m_driveVelocity_P)
        {
            driveMotorControllerConfig += " Pv";
        }
        if (m_drivePID->GetI(1) != m_driveVelocity_I)
        {
            driveMotorControllerConfig += " Iv";
        }
        if (m_drivePID->GetIZone(1) != m_driveVelocity_IZ)
        {
            driveMotorControllerConfig += " IZv";
        }
        if (m_drivePID->GetIMaxAccum(1) != m_driveVelocity_IM)
        {
            driveMotorControllerConfig += " IMv";
        }
        if (m_drivePID->GetD(1) != m_driveVelocity_D)
        {
            driveMotorControllerConfig += " Dv";
        }
        if (m_drivePID->GetDFilter(1) != m_driveVelocity_DF)
        {
            driveMotorControllerConfig += " DFv";
        }
        if (m_drivePID->GetFF(1) != m_driveVelocity_F)
        {
            driveMotorControllerConfig += " Fv";
        }
        if (m_drivePID->GetSmartMotionMaxVelocity(1) != m_driveVelocity_V)
        {
            driveMotorControllerConfig += " Vv";
        }
        if (m_drivePID->GetSmartMotionMaxAccel(1) != m_driveVelocity_A)
        {
            driveMotorControllerConfig += " Av";
        }

        if (driveMotorControllerConfig.empty())
        {
            m_driveMotorControllerConfig.clear();

            return true;
        }

        if (driveMotorControllerConfig != m_driveMotorControllerConfig)
        {
            printf("%s Drive Motor Config:%s\n", m_name.c_str(), driveMotorControllerConfig.c_str());

            m_driveMotorControllerConfig.swap(driveMotorControllerConfig);
        }

        return false; });
}

void SwerveModule::CreateTurningMotorControllerConfig() noexcept
{
    std::fprintf(stderr, "Swerve Module (%s) Turning Configuration... ", m_name.c_str());

    DoSafeTurningMotor("CreateTurningMotorControllerConfig()", [&]() -> void
                       {
        if (!m_turningMotor)
        {
            throw std::runtime_error("m_turningMotor");
        }

        if (m_turningMotor->RestoreFactoryDefaults() != rev::REVLibError::kOk)
        {
            throw std::runtime_error("RestoreFactoryDefaults()");
        } });

    std::fprintf(stderr, "Restore... ");
    if (!m_turningMotor)
    {
        std::fprintf(stderr, "FAILED.\n");

        return;
    }

    ConstructTurningMotor();

    std::fprintf(stderr, "New... ");
    if (!m_turningMotor)
    {
        std::fprintf(stderr, "FAILED.\n");

        return;
    }

    DoSafeTurningMotor("CreateTurningMotorControllerConfig()", [&]() -> void
                       {
        if (!m_turningMotor)
        {
            throw std::runtime_error("m_turningMotor");
        }

        if (m_turningMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIdleMode()");
        } });

    SetTurningPositionPID();

    std::fprintf(stderr, "Config... ");
    if (!m_turningMotor)
    {
        std::fprintf(stderr, "FAILED.\n");

        return;
    }

    DoSafeTurningMotor("CreateTurningMotorControllerConfig()", [&]() -> void
                       {
        if (!m_turningMotor)
        {
            throw std::runtime_error("m_turningMotor");
        }

        if (m_turningMotor->BurnFlash() != rev::REVLibError::kOk)
        {
            throw std::runtime_error("BurnFlash()");
        } });

    if (!m_turningMotor)
    {
        std::fprintf(stderr, "Save... FAILED.\n");

        return;
    }

    std::fprintf(stderr, " Saved... OK.\n");
}

void SwerveModule::CreateDriveMotorControllerConfig() noexcept
{
    std::fprintf(stderr, "Swerve Module (%s) Drive Configuration... ", m_name.c_str());

    DoSafeDriveMotor("CreateDriveMotorControllerConfig()", [&]() -> void
                     {
        if (!m_driveMotor)
        {
            throw std::runtime_error("m_driveMotor");
        }

        if (m_driveMotor->RestoreFactoryDefaults() != rev::REVLibError::kOk)
        {
            throw std::runtime_error("RestoreFactoryDefaults()");
        } });

    std::fprintf(stderr, "Restore... ");
    if (!m_driveMotor)
    {
        std::fprintf(stderr, "FAILED.\n");

        return;
    }

    ConstructDriveMotor();

    std::fprintf(stderr, "New... ");
    if (!m_driveMotor)
    {
        std::fprintf(stderr, "FAILED.\n");

        return;
    }

    DoSafeDriveMotor("CreateDriveMotorControllerConfig()", [&]() -> void
                     {
        if (!m_driveMotor)
        {
            throw std::runtime_error("m_driveMotor");
        }

        if (m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetIdleMode()");
        } });

    SetDrivePositionPID();

    SetDriveVelocityPID();

    std::fprintf(stderr, "Config... ");
    if (!m_driveMotor)
    {
        std::fprintf(stderr, "FAILED.\n");

        return;
    }

    DoSafeDriveMotor("CreateDriveMotorControllerConfig()", [&]() -> void
                     {
        if (!m_driveMotor)
        {
            throw std::runtime_error("m_driveMotor");
        }

        if (m_driveMotor->BurnFlash() != rev::REVLibError::kOk)
        {
            throw std::runtime_error("BurnFlash()");
        } });

    if (!m_driveMotor)
    {
        std::fprintf(stderr, "Save... FAILED.\n");

        return;
    }

    std::fprintf(stderr, " Saved... OK.\n");
}
