#pragma once

#include "infrastructure/SparkMax.h"

#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>

#include <chrono>
#include <memory>

class TransferArmSubsystem : public frc2::SubsystemBase
{
public:
    TransferArmSubsystem() noexcept;

    TransferArmSubsystem(const TransferArmSubsystem &) = delete;
    TransferArmSubsystem &operator=(const TransferArmSubsystem &) = delete;

    void Periodic() noexcept override;
    void DisableIntake() noexcept;
    void SetArmMotorVoltagePercent(const double percent) noexcept; // must be used in a PID loop to set arm position

private:
    std::unique_ptr<SmartMotorBase> m_ArmMotorBase;
    std::unique_ptr<SmartMotor<units::angle::turns>> m_ArmMotor;

#pragma region Utility
public:
    bool GetStatus() const noexcept;
    void BurnConfig() noexcept;
    void ClearFaults() noexcept;
#pragma endregion

#pragma region Test
public:
    void TestInit() noexcept;
    void TestExit() noexcept;
    void TestPeriodic() noexcept;
private:
    std::chrono::steady_clock::time_point m_verifyMotorControllersWhen;
#pragma endregion
};
