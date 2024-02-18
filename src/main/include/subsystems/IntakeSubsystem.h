#pragma once

#include "rev/CANSparkMax.h"

#include <frc/DoubleSolenoid.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

#include <chrono>
#include <memory>

class IntakeSubsystem : public frc2::SubsystemBase
{
public:
    IntakeSubsystem() noexcept;

    IntakeSubsystem(const IntakeSubsystem &) = delete;
    IntakeSubsystem &operator=(const IntakeSubsystem &) = delete;

    void StopIntake() noexcept;
    void SetSpinMotorVoltagePercent(const double percent) noexcept;
    
private:
    rev::CANSparkMax m_motor{intake::kIntakeSpinMotorCanID, rev::CANSparkMax::MotorType::kBrushed};
};
