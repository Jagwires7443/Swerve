#include "subsystems/IntakeSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

IntakeSubsystem::IntakeSubsystem() noexcept
{
    StopIntake();
}

void IntakeSubsystem::StopIntake() noexcept
{
    m_motor.StopMotor();
}

void IntakeSubsystem::SetSpinMotorVoltagePercent(const double percent) noexcept
{
    m_motor.SetVoltage(percent * 12_V);
}
