#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

ShooterSubsystem::ShooterSubsystem() noexcept
{
    const SmartMotorBase::ConfigMap config = {
        {"kStatus1", uint{250}},
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

void ShooterSubsystem::Set(double percent) noexcept
{
    m_shooterMotor->SetVoltage(percent * 12_V);
    // m_backspinMotor->SetVoltage(percent * 12_V);
}

void ShooterSubsystem::Stop() noexcept
{
    m_shooterMotor->Stop();
    m_backspinMotor->Stop();
}
