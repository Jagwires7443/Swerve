#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

// Need to derive from abstract Sendable class in order to be able to use the
// Gyro UI element in Shuffleboard; note that this doesn't actually derive from
// frc::Gyro or frc::GyroBase -- it's all down to inheritance and properties.
// Note that this makes use of the CRTP (Curiously Recurring Template Pattern).
class HeadingGyro : public wpi::Sendable, public wpi::SendableHelper<HeadingGyro>
{
public:
    HeadingGyro() noexcept = default;

    HeadingGyro(const HeadingGyro &) = delete;
    HeadingGyro &operator=(const HeadingGyro &) = delete;

    void InitSendable(wpi::SendableBuilder &builder) noexcept override
    {
        builder.SetSmartDashboardType("Gyro");
        builder.AddDoubleProperty(
            "Value", [&]() -> double
            { return m_value; },
            nullptr);
    }

    void Set(const double &value) noexcept { m_value = value; }

private:
    double m_value{0.0};
};

// Need to derive from abstract Sendable class in order to be able to use the
// PIDController UI element in Shuffleboard; note that this class doesn't
// derive from frc::PIDController -- it's all down to inheritance and
// properties.
class TuningPID : public wpi::Sendable, public wpi::SendableHelper<TuningPID>
{
public:
    TuningPID(double p, double i, double d, double f) noexcept : m_p(p), m_i(i), m_d(d), m_f(f) {}

    TuningPID(const TuningPID &) = delete;
    TuningPID &operator=(const TuningPID &) = delete;

    void InitSendable(wpi::SendableBuilder &builder) noexcept override
    {
        builder.SetSmartDashboardType("PIDController");
        builder.AddDoubleProperty(
            "p", [&]() -> double
            { return m_p; },
            [&](double value) -> void
            { m_p = value; });
        builder.AddDoubleProperty(
            "i", [&]() -> double
            { return m_i; },
            [&](double value) -> void
            { m_i = value; });
        builder.AddDoubleProperty(
            "d", [&]() -> double
            { return m_d; },
            [&](double value) -> void
            { m_d = value; });
        builder.AddDoubleProperty(
            "f", [&]() -> double
            { return m_f; },
            [&](double value) -> void
            { m_f = value; });
        builder.AddDoubleProperty(
            "setpoint", [&]() -> double
            { return m_s; },
            [&](double value) -> void
            { m_s = value; });
        builder.AddBooleanProperty(
            "enabled", [&]() -> bool
            { return m_e; },
            [&](bool value) -> void
            { m_e = value; });
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
