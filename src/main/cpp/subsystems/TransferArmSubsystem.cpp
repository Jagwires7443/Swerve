#include "subsystems/TransferArmSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>
#include <units/angle.h>
#include <frc/smartdashboard/SmartDashboard.h>

TransferArmSubsystem::TransferArmSubsystem() noexcept
{
    m_motor.RestoreFactoryDefaults();
    
    // set PID coefficients
    m_pidController.SetP(kP);
    m_pidController.SetI(kI);
    m_pidController.SetD(kD);
    m_pidController.SetIZone(kIz);
    m_pidController.SetFF(kFF);
    m_pidController.SetOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", kP);
    frc::SmartDashboard::PutNumber("I Gain", kI);
    frc::SmartDashboard::PutNumber("D Gain", kD);
    frc::SmartDashboard::PutNumber("I Zone", kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", kFF);
    frc::SmartDashboard::PutNumber("Max Output", kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", kMinOutput);
    frc::SmartDashboard::PutNumber("Set Rotations", 0);

    StopTransferArm();
}

void TransferArmSubsystem::StopTransferArm() noexcept
{
    m_motor.StopMotor();
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
    double p = frc::SmartDashboard::GetNumber("P Gain", 0);
    double i = frc::SmartDashboard::GetNumber("I Gain", 0);
    double d = frc::SmartDashboard::GetNumber("D Gain", 0);
    double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
    double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
    double max = frc::SmartDashboard::GetNumber("Max Output", 0);
    double min = frc::SmartDashboard::GetNumber("Min Output", 0);
    double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.SetP(p); kP = p; }
    if((i != kI)) { m_pidController.SetI(i); kI = i; }
    if((d != kD)) { m_pidController.SetD(d); kD = d; }
    if((iz != kIz)) { m_pidController.SetIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.SetFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.SetOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
}
