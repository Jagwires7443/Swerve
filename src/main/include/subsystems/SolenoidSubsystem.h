#pragma once

#include "Constants.h"
#include "rev/CANSparkMax.h"
#include <frc2/command/SubsystemBase.h>

class SolenoidSubsytem : public frc2::SubsystemBase
{
public:
    SolenoidSubsytem() noexcept;

    SolenoidSubsytem(const SolenoidSubsytem &) = delete;
    SolenoidSubsytem &operator=(const SolenoidSubsytem &) = delete;

    void SolenoidUp() noexcept;
    void SolenoidDown() noexcept;

private:
    
}