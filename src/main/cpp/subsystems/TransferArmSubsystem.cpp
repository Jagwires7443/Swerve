#include "subsystems/TransferArmSubsystem.h"
#include <units/voltage.h>

TransferArmSubsystem::TransferArmSubsystem() noexcept
{
    m_TransferArmMotor.SetInverted(arm::kTransferArmMotorIsInverted);

    m_encoder.SetMeasurementPeriod(10);

    StopTransferArm();
}

void TransferArmSubsystem::StopTransferArm() noexcept
{
    m_TransferArmMotor.StopMotor();
}

void TransferArmSubsystem::SetArmMotorVoltagePercent(const double percent) noexcept
{
    m_TransferArmMotor.SetVoltage(percent * 12_V);
}

units::turn_t TransferArmSubsystem::GetTransferArmPosition() noexcept
{
    return units::turn_t(m_encoder.GetPosition());
}
