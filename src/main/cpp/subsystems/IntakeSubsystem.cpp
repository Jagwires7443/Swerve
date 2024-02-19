#include "subsystems/IntakeSubsystem.h"
#include <units/voltage.h>

IntakeSubsystem::IntakeSubsystem() noexcept
{
    m_IntakeMotor.SetInverted(intake::kIntakeSpinMotorIsInverted);
    StopIntake();
}

void IntakeSubsystem::StopIntake() noexcept
{
    m_IntakeMotor.StopMotor();
}

void IntakeSubsystem::SetSpinMotorVoltagePercent(const double percent) noexcept
{
    m_IntakeMotor.SetVoltage(percent * 12_V);
}
