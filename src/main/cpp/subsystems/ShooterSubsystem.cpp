#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

ShooterSubsystem::ShooterSubsystem() noexcept
{
    // These motors use bang-bang control, thus an increased velocity reporting
    // rate (kStatus1).
    const SmartMotorBase::ConfigMap config = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{0}},
    };

    m_shooterMotorBase = SparkMaxFactory::CreateSparkMax("Shooter", 13, false);
    m_backspinMotorBase = SparkMaxFactory::CreateSparkMax("Backspin", 14, true);
    m_shooterMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_shooterMotorBase);
    m_backspinMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_backspinMotorBase);

    m_shooterMotor->SetConfig(config);
    m_backspinMotor->SetConfig(config);

    m_shooterMotor->ApplyConfig(false);
    m_backspinMotor->ApplyConfig(false);
}

void ShooterSubsystem::Periodic() noexcept
{
    m_shooterMotor->Periodic();
    m_backspinMotor->Periodic();
}

bool ShooterSubsystem::GetStatus() const noexcept
{
    return m_shooterMotor->GetStatus() ||
           m_backspinMotor->GetStatus();
}

void ShooterSubsystem::TestInit() noexcept {}
void ShooterSubsystem::TestExit() noexcept {}
void ShooterSubsystem::TestPeriodic() noexcept
{
    const std::chrono::time_point now = std::chrono::steady_clock::now();
    if (now >= m_verifyMotorControllersWhen)
    {
        using namespace std::chrono_literals;

        m_verifyMotorControllersWhen = now + 15s;

        m_shooterMotor->CheckConfig();
        m_backspinMotor->CheckConfig();
    }
}

void ShooterSubsystem::Default(const double percent, const double velocity) noexcept
{
    // Manual control (backup).
    if (velocity == 0.0)
    {
        m_shooterMotor->SetVoltage(percent * 12.00_V);
        m_backspinMotor->SetVoltage(percent * 11.25_V);

        return;
    }

    // On/off trigger control (off here).
    if (percent < 0.25)
    {
        m_shooterMotor->Stop();
        m_backspinMotor->Stop();

        return;
    }

// Unfortunately, SPARK MAX has too much velocity measurement latency for this.
#if 0
    // RPM.
    const double shooterVelocity = m_shooterMotor->GetVelocityRaw();
    // const double backspinVelocity = m_backspinMotor->GetVelocityRaw();

    // Bang-bang velocity control (simple).
    if (shooterVelocity < velocity)
    {
        m_shooterMotor->Set(1.0);
    }
    else
    {
        m_shooterMotor->Stop();
    }

#else

    m_shooterMotor->SetVoltage(velocity / 1500.0 * 12.00_V);
    m_backspinMotor->SetVoltage(velocity / 1500.0 * 11.25_V);

#endif
}

void ShooterSubsystem::Stop() noexcept
{
    m_shooterMotor->Stop();
    m_backspinMotor->Stop();
}

void ShooterSubsystem::BurnConfig() noexcept
{
    m_shooterMotor->ApplyConfig(true);
    m_backspinMotor->ApplyConfig(true);
}

void ShooterSubsystem::ClearFaults() noexcept
{
    m_shooterMotor->ClearFaults();
    m_backspinMotor->ClearFaults();
}
