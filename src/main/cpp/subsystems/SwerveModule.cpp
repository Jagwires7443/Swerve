// See https://github.com/wpilibsuite/allwpilib/blob/main/wpilibcExamples/src/main/cpp/examples/SwerveControllerCommand/cpp/subsystems/SwerveModule.cpp.
// See https://docs.revrobotics.com/sparkmax/operating-modes/closed-loop-control.

// XXX
// XXX file/update REV bug on wrapping issues; add code to do PID in code here, in case bug is there
// XXX
// XXX index pin forces wrap (in H/W); other than this quadrature encoders act as though they are on a number line
// XXX but, number lines in computers wrap (at some point, depends on number of bits and format)
// XXX this is analogous to drining around the world and wrapping, even though you think of just driving on
// XXX can do brief discussion on discrete rates/derivatives, but this is it for the H/W
// XXX signed/unsigned ints, mention float/double and cover int bit patterns
// XXX
// XXX swerve module has two common use cases for encoder data:
// XXX continuous/periodic; 1st is drive wheel, the number line scenario -- 2nd is steering, the periodic/wrapping scenario
// XXX linear -- do not want to wrap (ever), watch out for numeric overflow because it can happen
// XXX circular -- want to wrap at user-defined point (may not match encoder index/counts)
// XXX distance (since reference, e.g. zero or last check?) vs. position
// XXX velocity is always based on distance (but watch out for H/W wrap!), acceleration (used internally) is always based on velocity
// XXX but, if wrapping is done properly, distance and velocity will be the same either way
// XXX mention measurement of counts/time -- two ways, record time at count, or records counts at fixed in time sample points
// XXX
// XXX position reporting can always use distance (since can be fixed on roboRIO), position control must be circular though!
// XXX choice between unsigned/signed can be handled on roboRIO, or specify a range
// XXX (it would make sense to report wrapped position when in the given mode though)
// XXX circular position control based on error term -- need to compute two distances (on in each direction) and use smallest magnitude
// XXX in reality, want to *always* compute two distances and use the shorter -- just use H/W over/underflow wrap as default
// XXX

// XXX add using

// XXX init the module to home, do not do in test mode (do in auto/teleop init, via SetDesiredState()?)

// XXX explain error handling, consider CAN timeouts as non-fatal
// XXX consider other possible failure points
// XXX add comment on error handling strategy, update code to match

// XXX can set PID parameter to force updating of controllers -- or add a switch for this in DriveSubsystsem or overall robot, move there?

#include "subsystems/SwerveModule.h"

#include "Constants.h"

#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <networktables/NetworkTableValue.h>
#include <rev/CANError.h>
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

    m_rioPIDController = std::make_unique<frc2::PIDController>(
        pidf::kTurningPositionP,
        pidf::kTurningPositionI,
        pidf::kTurningPositionD);

    m_rioPIDController->EnableContinuousInput(-180.0, +180.0);

    // Construct turning absolute duty cycle encoder.  A `DigitalSource` is
    // required by the `DutyCycle` ctor.  Nothing here is expected to fail.
    // Note this has no explicit SetInverted() method; flip the turning motor
    // and it's incremental encoder in order to make things work out/line up.
    m_turningPositionSource = std::make_unique<frc::DigitalInput>(turningEncoderPort);
    m_turningPosition = std::make_unique<frc::DutyCycle>(*m_turningPositionSource);

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
    if (frequency < 208 || frequency > 280)
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
    auto position = GetAbsolutePosition(m_turningPosition->GetFrequency(),
                                        m_turningPosition->GetOutput(),
                                        true);

    if (!position.has_value())
    {
        return std::nullopt;
    }

    return units::angle::turn_t{static_cast<double>(position.value()) / 4096.0};
}

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
    bool result = false;

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
    bool result = false;

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

    DoSafeTurningMotor("ctor", [&]() -> void {
        m_turningMotor = std::make_unique<rev::CANSparkMax>(
            m_turningMotorCanID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        m_turningEncoder = std::make_unique<rev::CANEncoder>(
            m_turningMotor->GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 4096));
        m_turningPID = std::make_unique<rev::CANPIDController>(m_turningMotor->GetPIDController());

        if (!m_turningMotor)
        {
            throw std::runtime_error("m_turningMotor");
        }

        // Does not get sent to the motor controller, done locally.
        m_turningMotor->SetInverted(m_turningMotorInverted);

        if (m_turningMotor->ClearFaults() != rev::CANError::kOk)
        {
            throw std::runtime_error("ClearFaults()");
        }

        if (!m_turningEncoder || !m_turningPID)
        {
            throw std::runtime_error("m_turningEncoder");
        }

        if (m_turningEncoder->SetInverted(m_turningEncoderInverted) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetInverted()");
        }

        if (m_turningPID->SetFeedbackDevice(*m_turningEncoder) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetFeedbackDevice()");
        }
    });
}

// Construct and get/move drive motor objects.  It is possible for CAN or
// power issues to induce errors (particuarly from ctors), so do try/catch.
// The configuration is handled elsewhere; the expected case is that this has
// been saved, so that the motor controller comes up properly configured.

// This handles only those settings that are initially configured manually:
// kCanID (this cannot be configured programmatically), kMotorType/kSensorType,
// and kDataPortConfig; kIdleMode is not handled here).
void SwerveModule::ConstructDriveMotor() noexcept
{
    m_drivePID = nullptr;
    m_driveEncoder = nullptr;
    m_driveMotor = nullptr;

    DoSafeDriveMotor("ctor", [&]() -> void {
        m_driveMotor = std::make_unique<rev::CANSparkMax>(
            m_driveMotorCanID, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        m_driveEncoder = std::make_unique<rev::CANEncoder>(
            m_driveMotor->GetEncoder(rev::CANEncoder::EncoderType::kHallSensor, 42));
        m_drivePID = std::make_unique<rev::CANPIDController>(m_driveMotor->GetPIDController());

        if (!m_driveMotor)
        {
            throw std::runtime_error("m_driveMotor");
        }

        // Does not get sent to the motor controller, done locally.
        m_driveMotor->SetInverted(m_driveMotorInverted);

        if (m_driveMotor->ClearFaults() != rev::CANError::kOk)
        {
            throw std::runtime_error("ClearFaults()");
        }

        if (!m_driveEncoder || !m_drivePID)
        {
            throw std::runtime_error("m_driveEncoder");
        }

        if (m_drivePID->SetFeedbackDevice(*m_driveEncoder) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetFeedbackDevice()");
        }
    });
}

bool SwerveModule::GetStatus() noexcept
{
    return (GetAbsolutePosition().has_value()) &&
           m_turningMotor && m_turningMotorControllerValidated &&
           m_driveMotor && m_driveMotorControllerValidated;
}

// XXX
void SwerveModule::SetDriveBrakeMode(bool brake) noexcept {}

void SwerveModule::ResetTurning() noexcept
{
    m_rioPIDController->Reset();

    auto position = GetAbsolutePosition();

    if (position.has_value())
    {
        DoSafeTurningMotor("ResetTurning()", [&]() -> void {
            if (m_turningEncoder)
            {
                // Turning position is in rotations.  Any under/overflow should
                // be wrapped but, in any case, a reset will place things
                // within [-0.5, +0.5).
                if (m_turningEncoder->SetPosition(
                        units::angle::turn_t(position.value()).to<double>()) != rev::CANError::kOk)
                {
                    throw std::runtime_error("SetPosition()"); // XXX warn
                }
            }
        });
    }

    // If absolute position isn't available, there's no basis for resetting and
    // it's better to just let any old data stand.  This is when manual
    // pointing of the modules before a match could ensure a reasonable zero.
    // Of course, if anything is reset, etc. all bets are off -- still, leaving
    // things alone is the best that can be done under these circumstances.
}

void SwerveModule::ResetDrive() noexcept
{
    DoSafeDriveMotor("ResetDrive()", [&]() -> void {
        if (m_driveEncoder)
        {
            if (m_driveEncoder->SetPosition(0.0) != rev::CANError::kOk)
            {
                throw std::runtime_error("SetPosition()"); // XXX warn
            }
        }
    });
}

units::angle::degree_t SwerveModule::GetTurningPosition() noexcept
{
    auto position = GetAbsolutePosition();

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

    DoSafeTurningMotor("GetState()", [&]() -> void {
        if (m_turningEncoder)
        {
            encoderPosition = units::angle::turn_t{m_turningEncoder->GetPosition()};

            while (encoderPosition < 180_deg)
            {
                encoderPosition += 360_deg;
            }
            while (encoderPosition >= 180_deg)
            {
                encoderPosition -= 360_deg;
            }
        }
    });

    return encoderPosition;
}

void SwerveModule::Periodic(const bool setMotors) noexcept
{
    if (!m_rio)
    {
        return;
    }

    double calculated = m_rioPIDController->Calculate(GetTurningPosition().to<double>());

    // Feedforward is a form of open-loop control.  For turning, there is not
    // much to do, but add in a constant value based only on the direction of
    // any error.  This is essentially how much output is needed to get going
    // and can help to compensate for friction.
    if (calculated > 0)
    {
        calculated += m_rioPID_F;
    }
    else if (calculated < 0)
    {
        calculated -= m_rioPID_F;
    }

    if (!setMotors)
    {
        return;
    }

    DoSafeDriveMotor("Periodic()", [&]() -> void {
        if (m_turningMotor)
        {
            m_turningMotor->SetVoltage(calculated * 12_V);
        }
    });
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

    m_rioPIDController->SetSetpoint(adjustedPosition.to<double>());

    if (m_rio)
    {
        return;
    }

    DoSafeDriveMotor("SetTurningPosition()", [&]() -> void {
        if (m_turningPID)
        {
            if (m_turningPID->SetReference(
                    units::angle::turn_t(adjustedPosition).to<double>(), rev::kPosition) != rev::CANError::kOk)
            {
                throw std::runtime_error("SetReference()"); // XXX warn
            }
        }
    });
}

// Drive position and velocity are in rotations and rotations/second,
// respectively.

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

    DoSafeDriveMotor("GetDriveDistance()", [&]() -> void {
        if (m_driveEncoder)
        {
            result = m_driveEncoder->GetPosition() * physical::kDriveMetersPerRotation;
        }
    });

    return result;
}

void SwerveModule::SetDriveDistance(units::length::meter_t distance) noexcept
{
    DoSafeDriveMotor("SetDriveDistance()", [&]() -> void {
        if (m_drivePID)
        {
            if (m_drivePID->SetReference(
                    (distance / physical::kDriveMetersPerRotation).to<double>(), rev::kPosition) != rev::CANError::kOk)
            {
                throw std::runtime_error("SetReference()"); // XXX warn
            }
        }
    });
}

units::velocity::meters_per_second_t SwerveModule::GetDriveVelocity() noexcept
{
    units::velocity::meters_per_second_t result{0};

    DoSafeDriveMotor("GetDriveVelocity()", [&]() -> void {
        if (m_driveEncoder)
        {
            // GetVelocity() returns revolutions per minute, need meters per second
            result = m_driveEncoder->GetVelocity() * physical::kDriveMetersPerRotation / 60_s;
        }
    });

    return result;
}

void SwerveModule::SetDriveVelocity(units::velocity::meters_per_second_t velocity) noexcept
{
    DoSafeDriveMotor("SetDriveVelocity()", [&]() -> void {
        if (m_drivePID)
        {
            if (m_drivePID->SetReference(
                    (velocity * 60_s / physical::kDriveMetersPerRotation).to<double>(), rev::kVelocity, 1) != rev::CANError::kOk)
            {
                throw std::runtime_error("SetReference()"); // XXX warn
            }
        }
    });
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

    auto position = GetAbsolutePosition();

    if (position.has_value() && false) // XXX don't do this when debugging (maybe never in test mode?)
    {
        state = frc::SwerveModuleState::Optimize(
            referenceState, frc::Rotation2d(position.value()));
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
    std::printf("Swerve Module (%s) Test Mode Init... ", m_name.c_str());

    // It would be very nice if Shufflboard allowed specifying properties here:
    // "gridSize": 32.0
    // "showGrid": false
    // "hgap": 0.0
    // "vgap": 0.0
    // As it is, best results will be obtained when the defaults for new tabs
    // are set in this way, except that the defaults do not seem to be applied.
    frc::ShuffleboardTab &shuffleboardTab = frc::Shuffleboard::GetTab(m_name);

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

void SwerveModule::TestPeriodic(const bool setMotors) noexcept
{
    // Read controls information from Shuffleboard and manage interactive UI.
    bool zeroTurning = m_turningMotorReset->GetEntry().GetBoolean(false);
    double setTurning = m_turningMotorControl->GetEntry().GetDouble(0.0);
    bool zeroDrive = m_driveMotorReset->GetEntry().GetBoolean(false);
    double setDrive = m_driveMotorControl->GetEntry().GetDouble(0.0);

    if (zeroTurning)
    {
        m_rioPIDController->Reset();

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

    // Check motor controller configuration, but only infrequently!
    std::chrono::time_point now = std::chrono::steady_clock::now();
    if (now >= m_verifyMotorControllersWhen)
    {
        using namespace std::chrono_literals;

        m_verifyMotorControllersWhen = now + 10s;

        m_turningMotorControllerValidated = VerifyTurningMotorControllerConfig();
        m_driveMotorControllerValidated = VerifyDriveMotorControllerConfig();
    }

    // Obtain raw data from DutyCycle object; absolute position without
    // adding in any current alignment offset, at least here.
    int frequency = m_turningPosition->GetFrequency();
    double output = m_turningPosition->GetOutput();
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

    // Obtain raw data from turning CANSparkMax and set the output.
    temperature = 0.0;
    faults = 0;
    stickyFaults = 0;
    voltage = 0.0;
    current = 0.0;
    speed = 0.0;
    percent = 0.0;
    distance = 0.0;
    velocity = 0.0;

    DoSafeTurningMotor("TestPeriodic()", [&]() -> void {
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
                if (m_turningEncoder->SetPosition(0.0) != rev::CANError::kOk)
                {
                    throw std::runtime_error("SetPosition()");
                }
            }
        }

        if (m_turningMotor)
        {
            if (zeroTurning)
            {
                if (m_turningMotor->ClearFaults() != rev::CANError::kOk)
                {
                    throw std::runtime_error("ClearFaults()");
                }
            }

            if (setMotors)
            {
                m_turningMotor->Set(setTurning);
            }
        }
    });

    // Compute the discrepancy between the absolute encoder and the incremental
    // encoder (as seen by the motor controller).  This should be very small if
    // the module is not turning as the data is being collected.  However, if
    // the module is turning, some discrepany is expected.  For some of these
    // parameters, it may be useful to graph values over time, as the module is
    // rotated at constant angular velocity.  This would make it easy to find
    // the min/max or to spot any discontinuities, for example.
    double commandedHeading = m_commandedHeading.to<double>();
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

    // Obtain raw data from drive CANSparkMax and set the output.
    temperature = 0.0;
    faults = 0;
    stickyFaults = 0;
    voltage = 0.0;
    current = 0.0;
    speed = 0.0;
    percent = 0.0;
    distance = 0.0;
    velocity = 0.0;

    DoSafeDriveMotor("TestPeriodic()", [&]() -> void {
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
                if (m_driveEncoder->SetPosition(0.0) != rev::CANError::kOk)
                {
                    throw std::runtime_error("SetPosition()");
                }
            }
        }

        if (m_driveMotor)
        {
            if (zeroDrive)
            {
                if (m_driveMotor->ClearFaults() != rev::CANError::kOk)
                {
                    throw std::runtime_error("ClearFaults()");
                }
            }

            if (setMotors)
            {
                m_driveMotor->Set(setDrive);
            }
        }
    });

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

void SwerveModule::DrivePositionPID(double P, double I, double IZ, double IM, double D, double DF, double F) noexcept
{
    m_drivePosition_P = P;
    m_drivePosition_I = I;
    m_drivePosition_IZ = IZ;
    m_drivePosition_IM = IM;
    m_drivePosition_D = D;
    m_drivePosition_DF = DF;
    m_drivePosition_F = F;

    SetDrivePositionPID();
}

void SwerveModule::DriveVelocityPID(double P, double I, double IZ, double IM, double D, double DF, double F) noexcept
{
    m_driveVelocity_P = P;
    m_driveVelocity_I = I;
    m_driveVelocity_IZ = IZ;
    m_driveVelocity_IM = IM;
    m_driveVelocity_D = D;
    m_driveVelocity_DF = DF;
    m_driveVelocity_F = F;

    SetDriveVelocityPID();
}

void SwerveModule::SetTurningPositionPID() noexcept
{
    DoSafeTurningMotor("SetTurningPositionPID()", [&]() -> void {
        if (!m_turningPID)
        {
            throw std::runtime_error("m_turningPID"); // XXX warn
        }
        if (m_turningPID->SetP(m_turningPosition_P) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetP()"); // XXX warn
        }
        if (m_turningPID->SetI(m_turningPosition_I) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetI()"); // XXX warn
        }
        if (m_turningPID->SetIZone(m_turningPosition_IZ) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetIZone()"); // XXX warn
        }
        if (m_turningPID->SetIMaxAccum(m_turningPosition_IM) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetIMaxAccum()"); // XXX warn
        }
        if (m_turningPID->SetD(m_turningPosition_D) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetD()"); // XXX warn
        }
        if (m_turningPID->SetDFilter(m_turningPosition_DF) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetDFilter()"); // XXX warn
        }
        if (m_turningPID->SetFF(m_turningPosition_F) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetFF()"); // XXX warn
        }
    });
}

void SwerveModule::SetDrivePositionPID() noexcept
{
    DoSafeDriveMotor("SetDrivePositionPID()", [&]() -> void {
        if (!m_drivePID)
        {
            throw std::runtime_error("m_drivePID"); // XXX warn
        }
        if (m_drivePID->SetP(m_drivePosition_P) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetP()"); // XXX warn
        }
        if (m_drivePID->SetI(m_drivePosition_I) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetI()"); // XXX warn
        }
        if (m_drivePID->SetIZone(m_drivePosition_IZ) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetIZone()"); // XXX warn
        }
        if (m_drivePID->SetIMaxAccum(m_drivePosition_IM) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetIMaxAccum()"); // XXX warn
        }
        if (m_drivePID->SetD(m_drivePosition_D) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetD()"); // XXX warn
        }
        if (m_drivePID->SetDFilter(m_drivePosition_DF) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetDFilter()"); // XXX warn
        }
        if (m_drivePID->SetFF(m_drivePosition_F) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetFF()"); // XXX warn
        }
    });
}

void SwerveModule::SetDriveVelocityPID() noexcept
{
    DoSafeDriveMotor("SetDriveVelocityPID()", [&]() -> void {
        if (!m_drivePID)
        {
            throw std::runtime_error("m_drivePID"); // XXX warn
        }
        if (m_drivePID->SetP(m_driveVelocity_P, 1) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetP()"); // XXX warn
        }
        if (m_drivePID->SetI(m_driveVelocity_I, 1) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetI()"); // XXX warn
        }
        if (m_drivePID->SetIZone(m_driveVelocity_IZ, 1) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetIZone()"); // XXX warn
        }
        if (m_drivePID->SetIMaxAccum(m_driveVelocity_IM, 1) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetIMaxAccum()"); // XXX warn
        }
        if (m_drivePID->SetD(m_driveVelocity_D, 1) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetD()"); // XXX warn
        }
        if (m_drivePID->SetDFilter(m_driveVelocity_DF, 1) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetDFilter()"); // XXX warn
        }
        if (m_drivePID->SetFF(m_driveVelocity_F, 1) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetFF()"); // XXX warn
        }
    });
}

bool SwerveModule::VerifyTurningMotorControllerConfig() noexcept
{
    if (!m_turningMotor)
    {
        return false;
    }

    return DidSafeTurningMotor("VerifyTurningMotorControllerConfig()", [&]() -> bool {
        if (!m_turningMotor || !m_turningEncoder || !m_turningPID)
        {
            throw std::runtime_error("m_turningMotor"); // XXX warn
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

        return false;
    });
}

bool SwerveModule::VerifyDriveMotorControllerConfig() noexcept
{
    if (!m_driveMotor)
    {
        return false;
    }

    return DidSafeDriveMotor("VerifyDriveMotorControllerConfig()", [&]() -> bool {
        if (!m_driveMotor || !m_drivePID)
        {
            throw std::runtime_error("m_driveMotor"); // XXX warn
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
        // XXX Make sure this all hangs together...
        if (m_driveMotor->GetIdleMode() != (m_driveMotorBrake ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast))
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

        return false;
    });
}

void SwerveModule::CreateTurningMotorControllerConfig() noexcept
{
    std::fprintf(stderr, "Swerve Module (%s) Turning Configuration... ", m_name.c_str());

    DoSafeTurningMotor("CreateTurningMotorControllerConfig()", [&]() -> void {
        if (!m_turningMotor)
        {
            throw std::runtime_error("m_turningMotor"); // XXX warn
        }

        if (m_turningMotor->RestoreFactoryDefaults() != rev::CANError::kOk)
        {
            throw std::runtime_error("RestoreFactoryDefaults()"); // XXX warn
        }
    });

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

    DoSafeTurningMotor("CreateTurningMotorControllerConfig()", [&]() -> void {
        if (!m_turningMotor)
        {
            throw std::runtime_error("m_turningMotor"); // XXX warn
        }

        if (m_turningMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetIdleMode()"); // XXX warn
        }
    });

    SetTurningPositionPID();

    std::fprintf(stderr, "Config... ");
    if (!m_turningMotor)
    {
        std::fprintf(stderr, "FAILED.\n");

        return;
    }

    DoSafeTurningMotor("CreateTurningMotorControllerConfig()", [&]() -> void {
        if (!m_turningMotor)
        {
            throw std::runtime_error("m_turningMotor"); // XXX warn
        }

        if (m_turningMotor->BurnFlash() != rev::CANError::kOk)
        {
            throw std::runtime_error("BurnFlash()"); // XXX warn
        }
    });

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

    DoSafeDriveMotor("CreateDriveMotorControllerConfig()", [&]() -> void {
        if (!m_driveMotor)
        {
            throw std::runtime_error("m_driveMotor"); // XXX warn
        }

        if (m_driveMotor->RestoreFactoryDefaults() != rev::CANError::kOk)
        {
            throw std::runtime_error("RestoreFactoryDefaults()"); // XXX warn
        }
    });

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

    DoSafeDriveMotor("CreateDriveMotorControllerConfig()", [&]() -> void {
        if (!m_driveMotor)
        {
            throw std::runtime_error("m_driveMotor"); // XXX warn
        }

        if (m_driveMotor->SetIdleMode(rev::CANSparkMax::IdleMode::kCoast) != rev::CANError::kOk)
        {
            throw std::runtime_error("SetIdleMode()"); // XXX warn
        }
    });

    SetDrivePositionPID();

    SetDriveVelocityPID();

    std::fprintf(stderr, "Config... ");
    if (!m_driveMotor)
    {
        std::fprintf(stderr, "FAILED.\n");

        return;
    }

    DoSafeDriveMotor("CreateDriveMotorControllerConfig()", [&]() -> void {
        if (!m_driveMotor)
        {
            throw std::runtime_error("m_driveMotor"); // XXX warn
        }

        if (m_driveMotor->BurnFlash() != rev::CANError::kOk)
        {
            throw std::runtime_error("BurnFlash()"); // XXX warn
        }
    });

    if (!m_driveMotor)
    {
        std::fprintf(stderr, "Save... FAILED.\n");

        return;
    }

    std::fprintf(stderr, " Saved... OK.\n");
}
