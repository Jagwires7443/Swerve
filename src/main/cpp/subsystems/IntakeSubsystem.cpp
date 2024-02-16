#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

IntakeSubsystem::IntakeSubsystem() noexcept
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

    const SmartMotorBase::ConfigMap moreTorque = {
        {"kSmartCurrentStallLimit", uint{30}}, // Amps
    };

    m_ArmMotorBase = SparkFactory::CreateSparkMax(intake::kIntakeArmMotorName, intake::kIntakeArmMotorCanID, intake::kIntakeArmMotorIsInverted);
    m_SpinMotorBase = SparkFactory::CreateSparkMax(intake::kIntakeSpinMotorName, intake::kIntakeSpinMotorCanID, intake::kIntakeSpinMotorIsInverted);
    m_ArmMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_ArmMotorBase);
    m_SpinMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_SpinMotorBase);

    m_ArmMotor->SetConfig(config);
    m_SpinMotor->SetConfig(config);
    m_SpinMotor->AddConfig(moreTorque);

    DisableIntake();
}

void IntakeSubsystem::Periodic() noexcept
{
    m_ArmMotor->Periodic();
    m_SpinMotor->Periodic();
}

void IntakeSubsystem::DisableIntake() noexcept
{
    m_ArmMotor->Stop();
    m_SpinMotorBase->Stop();
}

void IntakeSubsystem::SetSpinMotorVoltagePercent(const double percent) noexcept
{
    m_SpinMotor->SetVoltage(percent * 12_V);
}

void IntakeSubsystem::SetArmMotorVoltagePercent(const double percent) noexcept
{
    m_ArmMotor->SetVoltage(percent * 12_V);
}

#pragma region Utility
bool IntakeSubsystem::GetStatus() const noexcept
{
    return m_ArmMotor->GetStatus() ||
           m_SpinMotor->GetStatus();
}

void IntakeSubsystem::BurnConfig() noexcept
{
    m_ArmMotor->ApplyConfig(true);
    m_SpinMotor->ApplyConfig(true);
}

void IntakeSubsystem::ClearFaults() noexcept
{
    m_ArmMotor->ClearFaults();
    m_SpinMotor->ClearFaults();
}
#pragma endregion

#pragma region Test
void IntakeSubsystem::TestInit() noexcept {}

void IntakeSubsystem::TestExit() noexcept {}

void IntakeSubsystem::TestPeriodic() noexcept
{
    const std::chrono::time_point now = std::chrono::steady_clock::now();
    if (now >= m_verifyMotorControllersWhen)
    {
        using namespace std::chrono_literals;

        m_verifyMotorControllersWhen = now + 15s;

        // m_motor->CheckConfig();
    }
}
#pragma endregion
