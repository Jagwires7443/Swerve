#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/SubsystemBase.h>
//#include <chrono>
//#include <memory>

class ShooterSubsystem : public frc2::SubsystemBase
{
public:
    ShooterSubsystem() noexcept;

    ShooterSubsystem(const ShooterSubsystem &) = delete;
    ShooterSubsystem &operator=(const ShooterSubsystem &) = delete;

    void StopShooter() noexcept;
    void SetShooterMotorVoltagePercent(const double percent) noexcept;

private:
    rev::CANSparkMax m_LeftShooterMotor{shooter::kLeftShooterMotorCanID, rev::CANSparkMax::MotorType::kBrushed};
    rev::CANSparkMax m_RightShooterMotor{shooter::kRightShooterMotorCanID, rev::CANSparkMax::MotorType::kBrushed};
};
