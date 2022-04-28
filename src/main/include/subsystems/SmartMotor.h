#pragma once

#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <units/angle.h>
#include <units/base.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <variant>

// This is a C++ interface; it has only public pure virtual member functions,
// no copy/move, and no data members.  Instances are obtained via a derived
// class and/or some factory.  See also the templated SmartMotor class below.
class SmartMotorBase
{
protected:
    // Derived classes provide a constructor similar to this one:
    // VendorSmartMotor(const int CanId, const int encoderCounts = 0) noexcept;

    // The expectation is that if `encoderCounts` is defaulted (or specified as
    // 0) this indicates that no external encoder is present (in this case, the
    // built-in encoder is used); if not, `encoderCounts` specifies number of
    // counts in one full rotation (possibly after mechanical power transmision
    // through gears, belts, chains, etc.).  Connection of such an encoder to
    // the motor controller obviously depends on the vendor specifics.  Any
    // subclass is, of course, free to use whatever parameters are appropriate.

    SmartMotorBase() noexcept = default;

public:
    virtual ~SmartMotorBase() noexcept = default;

    SmartMotorBase(const SmartMotorBase &) = delete;
    SmartMotorBase &operator=(const SmartMotorBase &) = delete;

    SmartMotorBase(SmartMotorBase &&) noexcept = default;
    SmartMotorBase &operator=(SmartMotorBase &&) noexcept = default;

    // This displays a diagnostic and control panel (likely only used in test
    // mode).  Provides basic control of motor and sends telemetry of all major
    // parameters.  Alternatively, this might be specified using wpi::Sendable.

    // A typical usage would be to call this from TestInit() and then to call
    // Periodic() reguarly -- i.e. from Subsystem::Periodic().  Periodic() will
    // only do extra work to update Shuffleboard after ShuffleboardCreate() has
    // been called.  The display panel occupies 20x6 tiles in Shuffleboard.
    // There is no undo for creation, updates will continue until the program
    // ends.  This behavior allows data to be monitored in all modes, so long
    // as test mode has run, which never happens in a normal match.

    // If supplied, control() will be called from Periodic() once each time the
    // "control" user interface element has a new value (starting from zero and
    // between -1.0 and 1.0, inclusive).  If supplied, reset() will be called
    // from Periodic() each time the "reset" user interface element is clicked.
    // No matter what, clicking "reset" will call Stop().  If not supplied, a
    // default control() will be used.  The same applies for reset().

    // The default control() simply calls Set() with the given value, meaning
    // percent control.  Other types of control require supplying the control()
    // parameter.  The default reset() calls SpecifyPosition(0.0) and
    // ClearFaults().
    virtual void ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                    std::function<void(double)> control = nullptr,
                                    std::function<void()> reset = nullptr) noexcept = 0;

    using ConfigValue = std::variant<bool, uint, double>;
    using ConfigMap = std::map<std::string, ConfigValue>;

    virtual void SetConfig(const ConfigMap config) noexcept = 0;

    virtual void AddConfig(const ConfigMap config) noexcept = 0;

    // Report status of last complete check of config information.  This call
    // initiates such a check -- the actual reporting happens via GetStatus()
    // -- which will include the results of the last check, if any such check
    // has completed (or run far enough to find a problem).
    virtual void CheckConfig() noexcept = 0;

    // Initiate saving of current config information; this causes GetStatus()
    // to report false until this has completed and read back for verification.
    // In other words, ApplyConfig() is a proper superset of CheckConfig().
    virtual void ApplyConfig(bool burn) noexcept = 0;

    // Because reading or writing require round-trip CAN messaging, it can be
    // slow, particuarly when a lot of this is required.  Additionally, it is
    // important to be able to manage CAN bus bandwidth by spreading such
    // messaging in time.  Therefore, configuration work happens in the
    // background, in small increments.  This is done by performing messaging
    // in Periodic().  Additionally, because there may be many devices being
    // configured, the CAN ID may be used to further spread thinga out.  This
    // may involve apportioning messaging based on where the given CAN ID falls
    // within the address space of CAN IDs (6 bits; may have a shorter period).

    // To keep Periodic() lightweight, management of config parameters is meant
    // to be done only following a call to CheckConfig() or ApplyConfig().  And
    // such calls are generally meant to be done in either Disabled (init) or
    // Test (init and interactive -- burning should only be done at interactive
    // and explicit request).  Since apply includes a post-application check,
    // one reasonable usage is to call ApplyConfig(false) from DisabledInit()
    // and to use ApplyConfig() from TestPeriodic(), in respose to user actions
    // such as updating PID parameters or activating some "burn" control.

    // Ideally, the desired configuration is permanently saved so that it comes
    // up after any power outage.  Again, any permanent update ("burn") used to
    // set this up should only be done under manual control.  This is where
    // Test mode comes in.  Disabled mode normally runs before Autonomous and
    // Teleop, so it may be used to provide some coverage in case settings have
    // not been permanently saved.  All of this replaces manual configuration
    // via vendor utilities and/or writing code to apply configuration settings
    // programatically.  Some settings (such as coast/brake) are more dynamic
    // and may be altered programmatically, but still have default settings
    // managed via this config mechanism.  It is also anticipated that Disabled
    // mode is where summary reporting of things such as cumulative faults,
    // controller restart counts, config issues, etc. will be done, as this
    // normally runs after Autonomous and Teleop.  Test mode is the only mode
    // where it is appropriate to be very verbose at all, as console spew can
    // cause issues and consumes resources.

    // Lightweight function which manages any faults or recovery, particuarly
    // detecting and handling controller restarts (reset or loss of power).
    // It also will do work to update Shuffleboard, and/or to manage config
    // parameters, but only when this has been requested (via other API calls).
    virtual void Periodic() noexcept = 0;

    // This likely has the side-effect of printing out a summary of any error
    // status.  As such, it is most appropriate to call sparingly, perhaps from
    // DisabledInit(), since this is normally called after Autonomous and
    // Teleop.  There's little reason to use this, apart from the summary info.
    virtual void ClearFaults() noexcept = 0;

    virtual bool GetStatus() noexcept = 0;

    enum class IdleMode
    {
        kCoast = 0,
        kBrake = 1,
    };

    virtual void SetIdleMode(const IdleMode mode) noexcept = 0;

    virtual IdleMode GetIdleMode() noexcept = 0;

    enum class Direction
    {
        kForward = 0,
        kReverse = 1,
    };

    virtual void EnableLimit(const Direction direction) noexcept = 0;

    virtual void DisableLimit(const Direction direction) noexcept = 0;

    virtual bool GetLimit(const Direction direction) noexcept = 0;

    virtual void Stop() noexcept = 0;

    virtual void Set(const double percent) noexcept = 0;

    virtual double Get() noexcept = 0;

    virtual void SetVoltage(const units::volt_t voltage) noexcept = 0;

    virtual void SetCurrent(const units::ampere_t current) noexcept = 0;

    // Used to establish zero position.
    virtual void SpecifyPosition(const double position) noexcept = 0;

    virtual void SeekPosition(const double position) noexcept = 0;

    virtual bool CheckPosition(const double tolerance) noexcept = 0;

    virtual double GetPositionRaw() noexcept = 0;

    virtual void SeekVelocity(const double velocity) noexcept = 0;

    virtual bool CheckVelocity(const double tolerance) noexcept = 0;

    virtual double GetVelocityRaw() noexcept = 0;
};

// This class offers the above interface and is templated on the metric for
// position (which influences velocity and acceleration).  The intent is that
// any smart motor can be represented by an instance of this class.

// The template parameter must be either an angle_unit or a length_unit.
template <typename Position>
class SmartMotor : private SmartMotorBase
{
public:
    using Position_t = units::unit_t<Position>;
    using Velocity = units::compound_unit<Position, units::inverse<units::seconds>>;
    using Velocity_t = units::unit_t<Velocity>;

    static_assert(units::traits::is_convertible_unit<Position_t, units::category::angle_unit>::value ||
                      units::traits::is_convertible_unit<Position_t, units::category::length_unit>::value,
                  "SmartMotor must be templated on either an angle_unit or a length_unit.");

    // Non-owning reference.
    explicit SmartMotor(SmartMotorBase &base) noexcept : base_{base} {}

    SmartMotor(const SmartMotor &) = delete;
    SmartMotor &operator=(const SmartMotor &) = delete;

    SmartMotor(SmartMotor &&) noexcept = default;
    SmartMotor &operator=(SmartMotor &&) noexcept = default;

    void ShuffleboardCreate(frc::ShuffleboardContainer &container,
                            std::function<void(double)> control = nullptr,
                            std::function<void()> reset = nullptr) noexcept override
    {
        base_.ShuffleboardCreate(container, control, reset);
    }

    void SetConfig(const ConfigMap config) noexcept override
    {
        base_.SetConfig(config);
    }

    void AddConfig(const ConfigMap config) noexcept override
    {
        base_.AddConfig(config);
    }

    void CheckConfig() noexcept override
    {
        base_.CheckConfig();
    }

    void ApplyConfig(bool burn) noexcept override
    {
        base_.ApplyConfig(burn);
    }

    void Periodic() noexcept override
    {
        base_.Periodic();
    }

    void ClearFaults() noexcept override
    {
        base_.ClearFaults();
    }

    bool GetStatus() noexcept override
    {
        return base_.GetStatus();
    }

    void SetIdleMode(const IdleMode mode) noexcept override
    {
        base_.SetIdleMode(mode);
    }

    IdleMode GetIdleMode() noexcept override
    {
        return base_.GetIdleMode();
    }

    void EnableLimit(const Direction direction) noexcept override
    {
        base_.EnableLimit(direction);
    }

    void DisableLimit(const Direction direction) noexcept override
    {
        base_.DisableLimit(direction);
    }

    bool GetLimit(const Direction direction) noexcept override
    {
        return base_.GetLimit(direction);
    }

    void Stop() noexcept override
    {
        base_.Stop();
    }

    void Set(const double percent) noexcept override
    {
        base_.Set(percent);
    }

    double Get() noexcept override
    {
        return base_.Get();
    }

    void SetVoltage(const units::volt_t voltage) noexcept override
    {
        base_.SetVoltage(voltage);
    }

    void SetCurrent(const units::ampere_t current) noexcept override
    {
        base_.SetCurrent(current);
    }

    void SpecifyPosition(const Position_t position) noexcept
    {
        base_.SpecifyPosition(position.value());
    }

    void SeekPosition(const Position_t position) noexcept
    {
        base_.SeekPosition(position.value());
    }

    bool CheckPosition(const Position_t tolerance) noexcept
    {
        return base_.CheckPosition(tolerance.value());
    }

    Position_t GetPosition() noexcept
    {
        return Position_t{base_.GetPositionRaw()};
    }

    void SeekVelocity(const Velocity_t velocity) noexcept
    {
        base_.SeekVelocity(velocity.value());
    }

    bool CheckVelocity(const Velocity_t tolerance) noexcept
    {
        return base_.CheckVelocity(tolerance.value());
    }

    Velocity_t GetVelocity() noexcept
    {
        return Velocity_t{base_.GetVelocityRaw()};
    }

protected:
    void SpecifyPosition(const double position) noexcept override
    {
        base_.SpecifyPosition(position);
    }

    void SeekPosition(const double position) noexcept override
    {
        base_.SeekPosition(position);
    }

    bool CheckPosition(const double tolerance) noexcept override
    {
        return base_.CheckPosition(tolerance);
    }

    double GetPositionRaw() noexcept override
    {
        return base_.GetPositionRaw();
    }

    void SeekVelocity(const double velocity) noexcept override
    {
        base_.SeekVelocity(velocity);
    }

    bool CheckVelocity(const double tolerance) noexcept override
    {
        return base_.CheckVelocity(tolerance);
    }

    double GetVelocityRaw() noexcept override
    {
        return base_.GetVelocityRaw();
    }

private:
    SmartMotorBase &base_;
};
