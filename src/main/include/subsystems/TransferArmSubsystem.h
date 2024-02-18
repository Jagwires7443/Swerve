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

    void StopIntake() noexcept;
    void SetArmMotorVoltagePercent(const double percent) noexcept; // must be used in a PID loop to set arm position

private:
    rev::CANSparkMax m_motor{intake::kIntakeArmMotorCanID, rev::CANSparkMax::MotorType::kBrushless};
};
