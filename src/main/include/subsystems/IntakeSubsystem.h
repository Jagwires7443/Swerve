#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/SubsystemBase.h>

class IntakeSubsystem : public frc2::SubsystemBase
{
public:
    IntakeSubsystem() noexcept;

    IntakeSubsystem(const IntakeSubsystem &) = delete;
    IntakeSubsystem &operator=(const IntakeSubsystem &) = delete;

    void StopIntake() noexcept;
    void SetSpinMotorVoltagePercent(const double percent) noexcept;
    
private:
    rev::CANSparkMax m_IntakeMotor{intake::kIntakeSpinMotorCanID, rev::CANSparkMax::MotorType::kBrushed};
};
