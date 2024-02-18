#include "subsystems/TransferArmSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

TransferArmSubsystem::TransferArmSubsystem() noexcept
{
    StopIntake();
}

void TransferArmSubsystem::StopIntake() noexcept
{
    m_motor.StopMotor();
}

void TransferArmSubsystem::SetArmMotorVoltagePercent(const double percent) noexcept
{
    m_motor.SetVoltage(percent * 12_V);
}
