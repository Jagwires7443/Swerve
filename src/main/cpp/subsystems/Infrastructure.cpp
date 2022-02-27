#include "subsystems/Infrastructure.h"

#include <units/pressure.h>

InfrastructureSubsystem::InfrastructureSubsystem() noexcept
{
    // XXX
    pdh_ = std::make_unique<frc::PowerDistribution>(1, frc::PowerDistribution::ModuleType::kRev);
    ph_ = std::make_unique<frc::Compressor>(1, frc::PneumaticsModuleType::REVPH);

    pdh_->ClearStickyFaults();
    // ph_->EnableAnalog(80_psi, 100_psi);
    ph_->Disable();
}

void InfrastructureSubsystem::Periodic() noexcept {}
