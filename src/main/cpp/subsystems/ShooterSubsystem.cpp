#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

ShooterSubsystem::ShooterSubsystem() noexcept
{
    // TODO: add comment here saying what motor this is for
    const SmartMotorBase::ConfigMap config = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{0}},
        {"kRampRate", double{0.1}},
        {"kSmartCurrentStallLimit", uint{25}}, // Amps
        {"kSmartCurrentFreeLimit", uint{10}}, 
    };

    m_LeftShooterMotorBase = SparkFactory::CreateSparkMax(
        shooter::kLeftShooterMotorName,
        shooter::kLeftShooterMotorCanID,
        shooter::kLeftShooterMotorIsInverted);
    m_RightShooterMotorBase = SparkFactory::CreateSparkMax(
        shooter::kRightShooterMotorName,
        shooter::kRightShooterMotorCanID,
        shooter::kRightShooterMotorIsInverted);
    m_LeftShooterMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_LeftShooterMotorBase);
    m_RightShooterMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_RightShooterMotorBase);

    m_LeftShooterMotor->SetConfig(config);
    m_RightShooterMotor->SetConfig(config);

    m_LeftShooterMotor->ApplyConfig(false);
    m_RightShooterMotor->ApplyConfig(false);

    Stop();
}

void ShooterSubsystem::Periodic() noexcept
{
    m_LeftShooterMotor->Periodic();
    m_RightShooterMotor->Periodic();
}

void ShooterSubsystem::SetShooterMotorVoltagePercent(const double percent) noexcept
{
    m_LeftShooterMotor->SetVoltage(percent * 12.00_V);
    m_RightShooterMotor->SetVoltage(percent * 12.00_V);
}

void ShooterSubsystem::Stop() noexcept
{
    m_LeftShooterMotor->Stop();
    m_RightShooterMotor->Stop();
}

#pragma region Utility
bool ShooterSubsystem::GetStatus() const noexcept
{
    return m_LeftShooterMotor->GetStatus() ||
           m_RightShooterMotor->GetStatus();
}

void ShooterSubsystem::BurnConfig() noexcept
{
    m_LeftShooterMotor->ApplyConfig(true);
    m_RightShooterMotor->ApplyConfig(true);
}

void ShooterSubsystem::ClearFaults() noexcept
{
    m_LeftShooterMotor->ClearFaults();
    m_RightShooterMotor->ClearFaults();
}
#pragma endregion

#pragma region Test
void ShooterSubsystem::TestInit() noexcept {}

void ShooterSubsystem::TestExit() noexcept {}

void ShooterSubsystem::TestPeriodic() noexcept
{
    const std::chrono::time_point now = std::chrono::steady_clock::now();
    if (now >= m_verifyMotorControllersWhen)
    {
        using namespace std::chrono_literals;

        m_verifyMotorControllersWhen = now + 15s;

        m_LeftShooterMotor->CheckConfig();
        m_RightShooterMotor->CheckConfig();
    }
}
#pragma endregion
