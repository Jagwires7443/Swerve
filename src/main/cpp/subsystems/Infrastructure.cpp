#include "subsystems/Infrastructure.h"

#include <utility>
#include <vector>

InfrastructureSubsystem::InfrastructureSubsystem() noexcept
{
    pdh_ = std::make_unique<frc::PowerDistribution>(1, frc::PowerDistribution::ModuleType::kRev);
    ph_ = std::make_unique<frc::Compressor>(1, frc::PneumaticsModuleType::REVPH);
    leds_ = std::make_unique<frc::Spark>(0);

    pdh_->ClearStickyFaults();
    pdh_->SetSwitchableChannel(false);

    Enable();
    SetLEDPattern(0);
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

units::pressure::pounds_per_square_inch_t InfrastructureSubsystem::GetPressure() noexcept
{
    return ph_->GetPressure();
}

// Copied from https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf.
const std::vector<std::pair<double, std::string_view>> LEDPatterns = {
    {-0.99, "Fixed Palette: Rainbow, Rainbow Palette [Density Speed Brightness]"},
    {-0.97, "Fixed Palette: Rainbow, Party Palette [Density Speed Brightness]"},
    {-0.95, "Fixed Palette: Rainbow, Ocean Palette [Density Speed Brightness]"},
    {-0.93, "Fixed Palette: Rainbow, Lave Palette [Density Speed Brightness]"},
    {-0.91, "Fixed Palette: Rainbow, Forest Palette [Density Speed Brightness]"},
    {-0.89, "Fixed Palette: Rainbow with Glitter [Density Speed Brightness]"},
    {-0.87, "Fixed Palette: Confetti [Density Speed Brightness]"},
    {-0.85, "Fixed Palette: Shot, Red [- - Brightness]"},
    {-0.83, "Fixed Palette: Shot, Blue [- - Brightness]"},
    {-0.81, "Fixed Palette: Shot, White [- - Brightness]"},
    {-0.79, "Fixed Palette: Sinelon, Rainbow Palette [Density Speed Brightness]"},
    {-0.77, "Fixed Palette: Sinelon, Party Palette [Density Speed Brightness]"},
    {-0.75, "Fixed Palette: Sinelon, Ocean Palette [Density Speed Brightness]"},
    {-0.73, "Fixed Palette: Sinelon, Lava Palette [Density Speed Brightness]"},
    {-0.71, "Fixed Palette: Sinelon, Forest Palette [Density Speed Brightness]"},
    {-0.69, "Fixed Palette: Beats per Minute, Rainbow Palette [Density Speed Brightness]"},
    {-0.67, "Fixed Palette: Beats per Minute, Party Palette [Density Speed Brightness]"},
    {-0.65, "Fixed Palette: Beats per Minute, Ocean Palette [Density Speed Brightness]"},
    {-0.63, "Fixed Palette: Beats per Minute, Lava Palette [Density Speed Brightness]"},
    {-0.61, "Fixed Palette: Beats per Minute, Forest Palette [Density Speed Brightness]"},
    {-0.59, "Fixed Palette: Fire, Medium [- - Brightness]"},
    {-0.57, "Fixed Palette: Fire, Large [- - Brightness]"},
    {-0.55, "Fixed Palette: Twinkles, Rainbow Palette [- - Brightness]"},
    {-0.53, "Fixed Palette: Twinkles, Party Palette [- - Brightness]"},
    {-0.51, "Fixed Palette: Twinkles, Ocean Palette [- - Brightness]"},
    {-0.49, "Fixed Palette: Twinkles, Lava Palette [- - Brightness]"},
    {-0.47, "Fixed Palette: Twinkles, Forest Palette [- - Brightness]"},
    {-0.45, "Fixed Palette: Color Waves, Rainbow Palette [- - Brightness]"},
    {-0.43, "Fixed Palette: Color Waves, Party Palette [- - Brightness]"},
    {-0.41, "Fixed Palette: Color Waves, Ocean Palette [- - Brightness]"},
    {-0.39, "Fixed Palette: Color Waves, Lava Palette [- - Brightness]"},
    {-0.37, "Fixed Palette: Color Waves, Forest Palette [- - Brightness]"},
    {-0.35, "Fixed Palette: Larson Scanner, Red [Width Speed Brightness]"},
    {-0.33, "Fixed Palette: Larson Scanner, Gray [Width Speed Brightness]"},
    {-0.31, "Fixed Palette: Light Chase, Red [Dimming Speed Brightness]"},
    {-0.29, "Fixed Palette: Light Chase, Blue [Dimming Speed Brightness]"},
    {-0.27, "Fixed Palette: Light Chase, Gray [Dimming Speed Brightness]"},
    {-0.25, "Fixed Palette: Heartbeat, Red [- - Brightness]"},
    {-0.23, "Fixed Palette: Heartbeat, Blue [- - Brightness]"},
    {-0.21, "Fixed Palette: Heartbeat, White [- - Brightness]"},
    {-0.19, "Fixed Palette: Heartbeat, Gray [- - Brightness]"},
    {-0.17, "Fixed Palette: Breath, Red [- - Brightness]"},
    {-0.15, "Fixed Palette: Breath, Blue [- - Brightness]"},
    {-0.13, "Fixed Palette: Breath, Gray [- - Brightness]"},
    {-0.11, "Fixed Palette: Strobe, Red [- - Brightness]"},
    {-0.09, "Fixed Palette: Strobe, Blue [- - Brightness]"},
    {-0.07, "Fixed Palette: Strobe, Gold [- - Brightness]"},
    {-0.05, "Fixed Palette: Strobe, White [- - Brightness]"},
    {-0.03, "Color 1: End to End Blend to Black [- - Brightness]"},
    {-0.01, "Color 1: Larson Scanner [Width Speed Brightness]"},
    {+0.01, "Color 1: Light Chase [Dimming Speed Brightness]"},
    {+0.03, "Color 1: Heartbeat Slow [- - Brightness]"},
    {+0.05, "Color 1: Heartbeat Medium [- - Brightness]"},
    {+0.07, "Color 1: Heartbeat Fast [- - Brightness]"},
    {+0.09, "Color 1: Breath Slow [- - Brightness]"},
    {+0.11, "Color 1: Breath Fast [- - Brightness]"},
    {+0.13, "Color 1: Shot [- - Brightness]"},
    {+0.15, "Color 1: Strobe [- - Brightness]"},
    {+0.17, "Color 2: End to End Blend to Black [- - Brightness]"},
    {+0.19, "Color 2: Larson Scanner [Width Speed Brightness]"},
    {+0.21, "Color 2: Light Chase [Dimming Speed Brightness]"},
    {+0.23, "Color 2: Heartbeat Slow [- - Brightness]"},
    {+0.25, "Color 2: Heartbeat Medium [- - Brightness]"},
    {+0.27, "Color 2: Heartbeat Fast [- - Brightness]"},
    {+0.29, "Color 2: Breath Slow [- - Brightness]"},
    {+0.31, "Color 2: Breath Fast [- - Brightness]"},
    {+0.33, "Color 2: Shot [- - Brightness]"},
    {+0.35, "Color 2: Strobe [- - Brightness]"},
    {+0.37, "Color 1 and 2: Sparkle, Color 1 on Color 2 [- - Brightness]"},
    {+0.39, "Color 1 and 2: Sparkle, Color 2 on Color 1 [- - Brightness]"},
    {+0.41, "Color 1 and 2: Color Gradient, Color 1 and 2 [- - Brightness]"},
    {+0.43, "Color 1 and 2: Beats per Minute, Color 1 and 2 [Density Speed Brightness]"},
    {+0.45, "Color 1 and 2: End to End Blend, Color 1 to 2 [- - Brightness]"},
    {+0.47, "Color 1 and 2: End to End Blend [- - Brightness]"},
    {+0.49, "Color 1 and 2: Color 1 and Color 2 no blending (Setup Pattern) [- - Brightness]"},
    {+0.51, "Color 1 and 2: Twinkles, Color 1 and 2 [- - Brightness]"},
    {+0.53, "Color 1 and 2: Color Waves, Color 1 and 2 [- - Brightness]"},
    {+0.55, "Color 1 and 2: Sinelon, Color 1 and 2 [Density Speed Brightness]"},
    {+0.57, "Solid Colors: Hot Pink [- - Brightness]"},
    {+0.59, "Solid Colors: Dark red [- - Brightness]"},
    {+0.61, "Solid Colors: Red [- - Brightness]"},
    {+0.63, "Solid Colors: Red Orange [- - Brightness]"},
    {+0.65, "Solid Colors: Orange [- - Brightness]"},
    {+0.67, "Solid Colors: Gold [- - Brightness]"},
    {+0.69, "Solid Colors: Yellow [- - Brightness]"},
    {+0.71, "Solid Colors: Lawn Green [- - Brightness]"},
    {+0.73, "Solid Colors: Lime [- - Brightness]"},
    {+0.75, "Solid Colors: Dark Green [- - Brightness]"},
    {+0.77, "Solid Colors: Green [- - Brightness]"},
    {+0.79, "Solid Colors: Blue Green [- - Brightness]"},
    {+0.81, "Solid Colors: Aqua [- - Brightness]"},
    {+0.83, "Solid Colors: Sky Blue [- - Brightness]"},
    {+0.85, "Solid Colors: Dark Blue [- - Brightness]"},
    {+0.87, "Solid Colors: Blue [- - Brightness]"},
    {+0.89, "Solid Colors: Blue Violet [- - Brightness]"},
    {+0.91, "Solid Colors: Violet [- - Brightness]"},
    {+0.93, "Solid Colors: White [- - Brightness]"},
    {+0.95, "Solid Colors: Gray [- - Brightness]"},
    {+0.97, "Solid Colors: Dark Gray [- - Brightness]"},
    {+0.99, "Solid Colors: Black [- - Brightness]"},
};

uint InfrastructureSubsystem::GetLEDPatternCount() noexcept
{
    return LEDPatterns.size();
}

std::string_view InfrastructureSubsystem::GetLEDPatternDescription(uint pattern) noexcept
{
    if (pattern >= LEDPatterns.size())
    {
        return "Bad Index!";
    }

    return LEDPatterns[pattern].second;
}

void InfrastructureSubsystem::SetLEDPattern(uint pattern) noexcept
{
    if (pattern >= LEDPatterns.size())
    {
        return;
    }

    leds_->Set(LEDPatterns[pattern].first);
}
