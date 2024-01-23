#pragma once

#include <frc/motorcontrol/Spark.h>
#include <frc/PowerDistribution.h>
#include <frc2/command/SubsystemBase.h>

#include <units/pressure.h>

#include <memory>
#include <string_view>

class InfrastructureSubsystem : public frc2::SubsystemBase
{
public:
    InfrastructureSubsystem() noexcept;

    InfrastructureSubsystem(const InfrastructureSubsystem &) = delete;
    InfrastructureSubsystem &operator=(const InfrastructureSubsystem &) = delete;

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    void Periodic() noexcept override;

    void Enable() noexcept;

    void Disable() noexcept;
};
