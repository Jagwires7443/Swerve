#include "subsystems/TransferArmSubsystem.h"
#include <units/voltage.h>
#include <frc/smartdashboard/SmartDashboard.h>

TransferArmSubsystem::TransferArmSubsystem() noexcept
{
    m_TransferArmMotor.SetInverted(arm::kTransferArmMotorIsInverted);

    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("Arm P Gain ", kP);
    frc::SmartDashboard::PutNumber("Arm D Gain ", kD);
    frc::SmartDashboard::PutNumber("Arm F ", kFF);

    StopTransferArm();
}

void TransferArmSubsystem::StopTransferArm() noexcept
{
    m_TransferArmMotor.StopMotor();
}

void TransferArmSubsystem::SetTransferArmPosition(const units::turn_t position) noexcept
{
    m_pidController.SetReference(position.value(), rev::CANSparkMax::ControlType::kPosition);
    frc::SmartDashboard::PutNumber("SetPoint", position.value());
    // frc::SmartDashboard::PutNumber("ProcessVariable", m_encoder.GetPosition());
}

// units::turn_t TransferArmSubsystem::GetTransferArmPosition() noexcept
// {
//     return units::turn_t(m_encoder.GetPosition());
// }

void TransferArmSubsystem::UpdatePIDValues() noexcept
{
    double p = frc::SmartDashboard::GetNumber("Arm P Gain ", 0);
    double d = frc::SmartDashboard::GetNumber("Arm D Gain ", 0);
    double ff = frc::SmartDashboard::GetNumber("Arm F ", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.SetP(p); kP = p; }
    if((d != kD)) { m_pidController.SetD(d); kD = d; }
    if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
}
