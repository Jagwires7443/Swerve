#include "subsystems/Infrastructure.h"

#include <units/pressure.h>

InfrastructureSubsystem::InfrastructureSubsystem() noexcept
{
    pdh_ = std::make_unique<frc::PowerDistribution>(1, frc::PowerDistribution::ModuleType::kRev);
    ph_ = std::make_unique<frc::Compressor>(1, frc::PneumaticsModuleType::REVPH);

    pdh_->ClearStickyFaults();
    pdh_->SetSwitchableChannel(false);

    Enable();
}

void InfrastructureSubsystem::Periodic() noexcept {}

void InfrastructureSubsystem::Enable() noexcept
{
    ph_->EnableAnalog(80_psi, 100_psi);
}

void InfrastructureSubsystem::Disable() noexcept
{
    ph_->Disable();
}
