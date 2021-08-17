#pragma once

#include "ctre/Phoenix.h"
#include <frc2/command/SubsystemBase.h>

#include <memory>

class FeederSubsystem : public frc2::SubsystemBase
{
public:
    // The only ctor of the FeederSubsystem class.
    FeederSubsystem() noexcept;

    FeederSubsystem(const FeederSubsystem &) = delete;
    FeederSubsystem &operator=(const FeederSubsystem &) = delete;

    void Set(double percent) noexcept;

private:
    void DoSafeFeederMotors(const char *const what, std::function<void()> work) noexcept;

    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX> m_feederOneMotor;
    std::unique_ptr<ctre::phoenix::motorcontrol::can::WPI_VictorSPX> m_feederTwoMotor;
};
