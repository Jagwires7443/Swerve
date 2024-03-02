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
    void SetArmMotorVoltagePercent(const double percent) noexcept;
    units::turn_t GetTransferArmPosition() noexcept;

private:
    rev::CANSparkMax m_TransferArmMotor{arm::kTransferArmMotorCanID, rev::CANSparkMax::MotorType::kBrushless};
    rev::SparkMaxAlternateEncoder m_encoder = m_TransferArmMotor.GetAlternateEncoder(8192);
};
