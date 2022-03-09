#pragma once

#include "subsystems/SparkMax.h"

#include <frc/RobotController.h>
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

    void Set(double percent) noexcept;

private:
    std::unique_ptr<SmartMotorBase> m_shooterMotor;

    uint64_t FPGATime_ = frc::RobotController::GetFPGATime();
    uint sequence_ = 0;
};
