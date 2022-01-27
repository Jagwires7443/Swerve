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

    void InitSendable(wpi::SendableBuilder &builder)
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
