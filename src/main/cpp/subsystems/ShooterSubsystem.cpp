#include "subsystems/ShooterSubsystem.h"
#include "subsystems/SparkMax.h"

#include "Constants.h"

#include <units/voltage.h>

ShooterSubsystem::ShooterSubsystem() noexcept
{
    m_shooterMotor = SparkMaxFactory::CreateSparkMax("Shooter", 1, false);
}

void ShooterSubsystem::Periodic() noexcept
{
    m_shooterMotor->Periodic();
}

void ShooterSubsystem::Set(double percent) noexcept
{
    m_shooterMotor->SetVoltage(percent * 12_V);
}
