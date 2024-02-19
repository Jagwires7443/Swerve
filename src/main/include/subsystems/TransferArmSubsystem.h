#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

class TransferArmSubsystem : public frc2::SubsystemBase
{
public:
    TransferArmSubsystem() noexcept;

    TransferArmSubsystem(const TransferArmSubsystem &) = delete;
    TransferArmSubsystem &operator=(const TransferArmSubsystem &) = delete;

    void StopTransferArm() noexcept;
    void SetTransferArmPosition(const units::turn_t position) noexcept;
    // units::turn_t GetTransferArmPosition() noexcept;
    void UpdatePIDValues() noexcept;

private:
    rev::CANSparkMax m_TransferArmMotor{arm::kTransferArmMotorCanID, rev::CANSparkMax::MotorType::kBrushless};
    // rev::CANEncoder m_encoder = m_TransferArmMotor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 8192);
    rev::SparkPIDController m_pidController = m_TransferArmMotor.GetPIDController();
    double kP = arm::kArmPositionP, kI = 0, kD = arm::kArmPositionD, kIz = 0, kFF = arm::kArmPositionF, kMaxOutput = 1, kMinOutput = -1;
};
