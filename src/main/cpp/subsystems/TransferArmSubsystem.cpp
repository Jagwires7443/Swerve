#include "subsystems/TransferArmSubsystem.h"
#include <units/voltage.h>
#include <frc/smartdashboard/SmartDashboard.h>

TransferArmSubsystem::TransferArmSubsystem() noexcept
{
    m_TransferArmMotor.SetInverted(arm::kTransferArmMotorIsInverted);

    m_encoder.SetMeasurementPeriod(10);

    frc::SmartDashboard::PutNumber("Arm Init Position ", m_encoder.GetPosition());

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
    frc::SmartDashboard::PutNumber("Arm Position ", m_encoder.GetPosition());
    return units::turn_t(m_encoder.GetPosition());
}
