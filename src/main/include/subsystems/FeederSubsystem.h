#pragma once

#include "infrastructure/SparkMax.h"

#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>

#include <chrono>
#include <memory>

class FeederSubsystem : public frc2::SubsystemBase
{
public:
    // The only ctor of the FeederSubsystem class.
    FeederSubsystem() noexcept;

    FeederSubsystem(const FeederSubsystem &) = delete;
    FeederSubsystem &operator=(const FeederSubsystem &) = delete;

    void Periodic() noexcept override;

    bool GetStatus() const noexcept;

    void TestInit() noexcept;
    void TestExit() noexcept;
    void TestPeriodic() noexcept;

    void Default(const double percent) noexcept;
    void NoFeed() noexcept;

    void Eject() noexcept;

    void Fire() noexcept;

    void Raise() noexcept;

    void Lower() noexcept;

    void LockIntake() noexcept;

    void DropIntake() noexcept;

    void RaiseIntake() noexcept;

    void LowerIntake() noexcept;

    void BurnConfig() noexcept;

    void ClearFaults() noexcept;

private:
    std::unique_ptr<SmartMotorBase> m_intakeMotorBase;
    std::unique_ptr<SmartMotorBase> m_elevatorMotorBase;
    std::unique_ptr<SmartMotorBase> m_feederMotorBase;
    std::unique_ptr<SmartMotorBase> m_climberMotorBase;
    std::unique_ptr<SmartMotor<units::angle::turns>> m_intakeMotor;
    std::unique_ptr<SmartMotor<units::angle::turns>> m_elevatorMotor;
    std::unique_ptr<SmartMotor<units::angle::turns>> m_feederMotor;
    std::unique_ptr<SmartMotor<units::angle::turns>> m_climberMotor;

    std::unique_ptr<frc::DoubleSolenoid> m_intakeRelease;
    std::unique_ptr<frc::DoubleSolenoid> m_intakeRaise;

    std::chrono::steady_clock::time_point m_verifyMotorControllersWhen;
};
