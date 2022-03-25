#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

ShooterSubsystem::ShooterSubsystem() noexcept
{
    // These motors use bang-bang control, thus an increased velocity reporting
    // rate (kStatus1).
    const SmartMotorBase::ConfigMap config = {
        {"kStatus1", uint{10}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{0}},
    };

    m_shooterMotor = SparkMaxFactory::CreateSparkMax("Shooter", 13, false);
    m_backspinMotor = SparkMaxFactory::CreateSparkMax("Backspin", 14, false);

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

void ShooterSubsystem::TestInit() noexcept {}
void ShooterSubsystem::TestExit() noexcept {}
void ShooterSubsystem::TestPeriodic() noexcept {}

void ShooterSubsystem::Default(const double percent, const double velocity) noexcept
{
#if 0 // XXX

    // RPM.
    const double shooterVelocity = m_shooterMotor->GetVelocityRaw();
    // const double backspinVelocity = m_backspinMotor->GetVelocityRaw();

    if (shooterVelocity < velocity)
    {
        m_shooterMotor->Set(1.0);
    }
    else
    {
        m_shooterMotor->Stop();
    }

#else

    if (velocity == 0.0)
    {
        m_shooterMotor->SetVoltage(percent * 12_V);
        // m_backspinMotor->SetVoltage(percent * 12_V);
    }
    else if (percent > 0.25)
    {
        m_shooterMotor->SetVoltage(velocity / 1500.0 * 12_V);
        // m_backspinMotor->SetVoltage(velocity / 1500.0 * 12_V);
    }
    else
    {
        m_shooterMotor->Stop();
        m_backspinMotor->Stop();
    }
#endif
}

void ShooterSubsystem::Stop() noexcept
{
    m_shooterMotor->Stop();
    m_backspinMotor->Stop();
}
