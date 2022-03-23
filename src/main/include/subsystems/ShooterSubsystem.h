#pragma once

#include "subsystems/SparkMax.h"

#include <frc2/command/SubsystemBase.h>

#include <memory>

class ShooterSubsystem : public frc2::SubsystemBase
{
public:
    // The only ctor of the FeederSubsystem class.
    ShooterSubsystem() noexcept;

    ShooterSubsystem(const ShooterSubsystem &) = delete;
    ShooterSubsystem &operator=(const ShooterSubsystem &) = delete;

    void Periodic() noexcept override;

    void Default(const double percent) noexcept;

    void Stop() noexcept;

private:
    std::unique_ptr<SmartMotorBase> m_shooterMotor;
    std::unique_ptr<SmartMotorBase> m_backspinMotor;
};
