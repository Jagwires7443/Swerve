#pragma once

#include <frc/Compressor.h>
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

    units::pressure::pounds_per_square_inch_t GetPressure() noexcept;

    uint GetLEDPatternCount() noexcept;

    std::string_view GetLEDPatternDescription(uint pattern) noexcept;

    void SetLEDPattern(uint pattern) noexcept;

private:
    std::unique_ptr<frc::PowerDistribution> pdh_;
    std::unique_ptr<frc::Compressor> ph_;
    std::unique_ptr<frc::Spark> leds_;
};
