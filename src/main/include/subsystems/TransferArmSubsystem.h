#pragma once

#include "rev/CANSparkMax.h"

#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

#include <chrono>
#include <memory>

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
    rev::CANSparkMax m_motor{intake::kIntakeArmMotorCanID, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkPIDController m_pidController = m_motor.GetPIDController();
    //rev::CANEncoder m_encoder = m_motor.GetAlternateEncoder(rev::CANEncoder::AlternateEncoderType::kQuadrature, 8192);
    double kP = 0.001, kI = 0, kD = 0, kIz = 0, kFF = 0, kMaxOutput = 1, kMinOutput = -1;
};
