#pragma once

#include "Constants.h"
#include "infrastructure/SparkMax.h"


#include <frc2/command/SubsystemBase.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include <chrono>
#include <memory>
#include <array>
#include <functional>
#include <optional>
#include <utility>

class ShooterSubsystem : public frc2::SubsystemBase
{
public:
    // The only ctor of the FeederSubsystem class.
    ShooterSubsystem() noexcept;

    ShooterSubsystem(const ShooterSubsystem &) = delete;
    ShooterSubsystem &operator=(const ShooterSubsystem &) = delete;

    void Periodic() noexcept override;

    bool GetStatus() const noexcept;

    void TestInit() noexcept;
    void TestExit() noexcept;
    void TestPeriodic() noexcept;

    void Default(const double percent, const double velocity) noexcept;

    void Stop() noexcept;

    void BurnConfig() noexcept;

    void ClearFaults() noexcept;

private:
    std::unique_ptr<SmartMotorBase> m_shooterMotorBase;
    std::unique_ptr<SmartMotorBase> m_backspinMotorBase;
    std::unique_ptr<SmartMotor<units::angle::turns>> m_shooterMotor;
    std::unique_ptr<SmartMotor<units::angle::turns>> m_backspinMotor;

    std::chrono::steady_clock::time_point m_verifyMotorControllersWhen;
};
