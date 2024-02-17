#include "subsystems/TransferArmSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

TransferArmSubsystem::TransferArmSubsystem() noexcept
{
    // TODO: add comment here saying what motor this is for
    const SmartMotorBase::ConfigMap config = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{0}},
        {"kRampRate", double{0.1}},
        {"kSmartCurrentStallLimit", uint{15}}, // Amps
        {"kSmartCurrentFreeLimit", uint{10}},  // Amps
    };

    m_ArmMotorBase = SparkFactory::CreateSparkMax(
        intake::kIntakeArmMotorName,
        intake::kIntakeArmMotorCanID,
        intake::kIntakeArmMotorIsInverted);
    m_ArmMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_ArmMotorBase);
    
    m_ArmMotor->SetConfig(config);
   
    DisableIntake();
}

void TransferArmSubsystem::Periodic() noexcept
{
    m_ArmMotor->Periodic();
}

void TransferArmSubsystem::DisableIntake() noexcept
{
    m_ArmMotor->Stop();
}


void TransferArmSubsystem::SetArmMotorVoltagePercent(const double percent) noexcept
{
    m_ArmMotor->SetVoltage(percent * 12_V);
}

#pragma region Utility
bool TransferArmSubsystem::GetStatus() const noexcept
{
    return m_ArmMotor->GetStatus();
}

void TransferArmSubsystem::BurnConfig() noexcept
{
    m_ArmMotor->ApplyConfig(true);
}

void TransferArmSubsystem::ClearFaults() noexcept
{
    m_ArmMotor->ClearFaults();
}
#pragma endregion

#pragma region Test
void TransferArmSubsystem::TestInit() noexcept {}

void TransferArmSubsystem::TestExit() noexcept {}

void TransferArmSubsystem::TestPeriodic() noexcept
{
    const std::chrono::time_point now = std::chrono::steady_clock::now();
    if (now >= m_verifyMotorControllersWhen)
    {
        using namespace std::chrono_literals;

        m_verifyMotorControllersWhen = now + 15s;

        m_ArmMotor->CheckConfig();
    }
}
#pragma endregion
