#include "subsystems/ShooterSubsystem.h"
#include <units/voltage.h>

ShooterSubsystem::ShooterSubsystem() noexcept
{
    m_LeftShooterMotor.SetInverted(shooter::kLeftShooterMotorIsInverted);
    m_RightShooterMotor.SetInverted(shooter::kRightShooterMotorIsInverted);
    StopShooter();
}

void ShooterSubsystem::StopShooter() noexcept
{
    m_LeftShooterMotor.StopMotor();
    m_RightShooterMotor.StopMotor();
}

void ShooterSubsystem::SetShooterMotorVoltagePercent(const double percent) noexcept
{
    m_LeftShooterMotor.SetVoltage(percent * 12.00_V);
    m_RightShooterMotor.SetVoltage(percent * 12.00_V);
}
