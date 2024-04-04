#include "subsystems/Infrastructure.h"

#include <utility>
#include <vector>

InfrastructureSubsystem::InfrastructureSubsystem() noexcept
{
    pdh_ = std::make_unique<frc::PowerDistribution>(1, frc::PowerDistribution::ModuleType::kRev);
    leds_ = std::make_unique<frc::Spark>(0);
    m_notesensor = std::make_unique<frc::DigitalInput>(nonDrive::kNoteSensor);
    Enable();
    SetLEDPattern(3);
    SetNumberLights(false);

    pdh_->ClearStickyFaults();
    pdh_->SetSwitchableChannel(true);
}

void InfrastructureSubsystem::Periodic() noexcept
{
    if (NoteSensor() == false)
    {
        printf("no note");
    }
    else
    {
        printf("yes note");
    }

    if (NoteSensor() == true)
    {
        SetLEDPattern(30);
    }

    if (NoteSensor() == false)
    {
        SetLEDPattern(36);
    }
}

void InfrastructureSubsystem::Enable() noexcept
{
}

const std::vector<std::pair<double, std::string_view>> LEDPatterns = {
    {-0.99, "Fixed Palette: Rainbow, Rainbow Palette [Density Speed Brightness]"},              // 1
    {-0.97, "Fixed Palette: Rainbow, Party Palette [Density Speed Brightness]"},                // 2
    {-0.95, "Fixed Palette: Rainbow, Ocean Palette [Density Speed Brightness]"},                // 3
    {-0.93, "Fixed Palette: Rainbow, Lave Palette [Density Speed Brightness]"},                 // 4
    {-0.91, "Fixed Palette: Rainbow, Forest Palette [Density Speed Brightness]"},               // 5
    {-0.89, "Fixed Palette: Rainbow with Glitter [Density Speed Brightness]"},                  // 6
    {-0.87, "Fixed Palette: Confetti [Density Speed Brightness]"},                              // 7
    {-0.85, "Fixed Palette: Shot, Red [- - Brightness]"},                                       // 8
    {-0.83, "Fixed Palette: Shot, Blue [- - Brightness]"},                                      // 9
    {-0.81, "Fixed Palette: Shot, White [- - Brightness]"},                                     // 10
    {-0.79, "Fixed Palette: Sinelon, Rainbow Palette [Density Speed Brightness]"},              // 11
    {-0.77, "Fixed Palette: Sinelon, Party Palette [Density Speed Brightness]"},                // 12
    {-0.75, "Fixed Palette: Sinelon, Ocean Palette [Density Speed Brightness]"},                // 13
    {-0.73, "Fixed Palette: Sinelon, Lava Palette [Density Speed Brightness]"},                 // 14
    {-0.71, "Fixed Palette: Sinelon, Forest Palette [Density Speed Brightness]"},               // 15
    {-0.69, "Fixed Palette: Beats per Minute, Rainbow Palette [Density Speed Brightness]"},     // 16
    {-0.67, "Fixed Palette: Beats per Minute, Party Palette [Density Speed Brightness]"},       // 17
    {-0.65, "Fixed Palette: Beats per Minute, Ocean Palette [Density Speed Brightness]"},       // 18
    {-0.63, "Fixed Palette: Beats per Minute, Lava Palette [Density Speed Brightness]"},        // 19
    {-0.61, "Fixed Palette: Beats per Minute, Forest Palette [Density Speed Brightness]"},      // 20
    {-0.59, "Fixed Palette: Fire, Medium [- - Brightness]"},                                    // 21
    {-0.57, "Fixed Palette: Fire, Large [- - Brightness]"},                                     // 22
    {-0.55, "Fixed Palette: Twinkles, Rainbow Palette [- - Brightness]"},                       // 23
    {-0.53, "Fixed Palette: Twinkles, Party Palette [- - Brightness]"},                         // 24
    {-0.51, "Fixed Palette: Twinkles, Ocean Palette [- - Brightness]"},                         // 25
    {-0.49, "Fixed Palette: Twinkles, Lava Palette [- - Brightness]"},                          // 26
    {-0.47, "Fixed Palette: Twinkles, Forest Palette [- - Brightness]"},                        // 27
    {-0.45, "Fixed Palette: Color Waves, Rainbow Palette [- - Brightness]"},                    // 28
    {-0.43, "Fixed Palette: Color Waves, Party Palette [- - Brightness]"},                      // 29
    {-0.41, "Fixed Palette: Color Waves, Ocean Palette [- - Brightness]"},                      // 30
    {-0.39, "Fixed Palette: Color Waves, Lava Palette [- - Brightness]"},                       // 31
    {-0.37, "Fixed Palette: Color Waves, Forest Palette [- - Brightness]"},                     // 32
    {-0.35, "Fixed Palette: Larson Scanner, Red [Width Speed Brightness]"},                     // 33
    {-0.33, "Fixed Palette: Larson Scanner, Gray [Width Speed Brightness]"},                    // 34
    {-0.31, "Fixed Palette: Light Chase, Red [Dimming Speed Brightness]"},                      // 35
    {-0.29, "Fixed Palette: Light Chase, Blue [Dimming Speed Brightness]"},                     // 36
    {-0.27, "Fixed Palette: Light Chase, Gray [Dimming Speed Brightness]"},                     // 37
    {-0.25, "Fixed Palette: Heartbeat, Red [- - Brightness]"},                                  // 38
    {-0.23, "Fixed Palette: Heartbeat, Blue [- - Brightness]"},                                 // 39
    {-0.21, "Fixed Palette: Heartbeat, White [- - Brightness]"},                                // 40
    {-0.19, "Fixed Palette: Heartbeat, Gray [- - Brightness]"},                                 // 41
    {-0.17, "Fixed Palette: Breath, Red [- - Brightness]"},                                     // 42
    {-0.15, "Fixed Palette: Breath, Blue [- - Brightness]"},                                    // 43
    {-0.13, "Fixed Palette: Breath, Gray [- - Brightness]"},                                    // 44
    {-0.11, "Fixed Palette: Strobe, Red [- - Brightness]"},                                     // 45
    {-0.09, "Fixed Palette: Strobe, Blue [- - Brightness]"},                                    // 46
    {-0.07, "Fixed Palette: Strobe, Gold [- - Brightness]"},                                    // 47
    {-0.05, "Fixed Palette: Strobe, White [- - Brightness]"},                                   // 48
    {-0.03, "Color 1: End to End Blend to Black [- - Brightness]"},                             // 49
    {-0.01, "Color 1: Larson Scanner [Width Speed Brightness]"},                                // 50
    {+0.01, "Color 1: Light Chase [Dimming Speed Brightness]"},                                 // 51
    {+0.03, "Color 1: Heartbeat Slow [- - Brightness]"},                                        // 52
    {+0.05, "Color 1: Heartbeat Medium [- - Brightness]"},                                      // 53
    {+0.07, "Color 1: Heartbeat Fast [- - Brightness]"},                                        // 54
    {+0.09, "Color 1: Breath Slow [- - Brightness]"},                                           // 55
    {+0.11, "Color 1: Breath Fast [- - Brightness]"},                                           // 56
    {+0.13, "Color 1: Shot [- - Brightness]"},                                                  // 57
    {+0.15, "Color 1: Strobe [- - Brightness]"},                                                // 58
    {+0.17, "Color 2: End to End Blend to Black [- - Brightness]"},                             // 59
    {+0.19, "Color 2: Larson Scanner [Width Speed Brightness]"},                                // 60
    {+0.21, "Color 2: Light Chase [Dimming Speed Brightness]"},                                 // 61
    {+0.23, "Color 2: Heartbeat Slow [- - Brightness]"},                                        // 62
    {+0.25, "Color 2: Heartbeat Medium [- - Brightness]"},                                      // 63
    {+0.27, "Color 2: Heartbeat Fast [- - Brightness]"},                                        // 64
    {+0.29, "Color 2: Breath Slow [- - Brightness]"},                                           // 65
    {+0.31, "Color 2: Breath Fast [- - Brightness]"},                                           // 66
    {+0.33, "Color 2: Shot [- - Brightness]"},                                                  // 67
    {+0.35, "Color 2: Strobe [- - Brightness]"},                                                // 68
    {+0.37, "Color 1 and 2: Sparkle, Color 1 on Color 2 [- - Brightness]"},                     // 69
    {+0.39, "Color 1 and 2: Sparkle, Color 2 on Color 1 [- - Brightness]"},                     // 70
    {+0.41, "Color 1 and 2: Color Gradient, Color 1 and 2 [- - Brightness]"},                   // 71
    {+0.43, "Color 1 and 2: Beats per Minute, Color 1 and 2 [Density Speed Brightness]"},       // 72
    {+0.45, "Color 1 and 2: End to End Blend, Color 1 to 2 [- - Brightness]"},                  // 73
    {+0.47, "Color 1 and 2: End to End Blend [- - Brightness]"},                                // 74
    {+0.49, "Color 1 and 2: Color 1 and Color 2 no blending (Setup Pattern) [- - Brightness]"}, // 75
    {+0.51, "Color 1 and 2: Twinkles, Color 1 and 2 [- - Brightness]"},                         // 76
    {+0.53, "Color 1 and 2: Color Waves, Color 1 and 2 [- - Brightness]"},                      // 77
    {+0.55, "Color 1 and 2: Sinelon, Color 1 and 2 [Density Speed Brightness]"},                // 78
    {+0.57, "Solid Colors: Hot Pink [- - Brightness]"},                                         // 79
    {+0.59, "Solid Colors: Dark red [- - Brightness]"},                                         // 80
    {+0.61, "Solid Colors: Red [- - Brightness]"},                                              // 81
    {+0.63, "Solid Colors: Red Orange [- - Brightness]"},                                       // 82
    {+0.65, "Solid Colors: Orange [- - Brightness]"},                                           // 83
    {+0.67, "Solid Colors: Gold [- - Brightness]"},                                             // 84
    {+0.69, "Solid Colors: Yellow [- - Brightness]"},                                           // 85
    {+0.71, "Solid Colors: Lawn Green [- - Brightness]"},                                       // 86
    {+0.73, "Solid Colors: Lime [- - Brightness]"},                                             // 87
    {+0.75, "Solid Colors: Dark Green [- - Brightness]"},                                       // 88
    {+0.77, "Solid Colors: Green [- - Brightness]"},                                            // 89
    {+0.79, "Solid Colors: Blue Green [- - Brightness]"},                                       // 90
    {+0.81, "Solid Colors: Aqua [- - Brightness]"},                                             // 91
    {+0.83, "Solid Colors: Sky Blue [- - Brightness]"},                                         // 92
    {+0.85, "Solid Colors: Dark Blue [- - Brightness]"},                                        // 93
    {+0.87, "Solid Colors: Blue [- - Brightness]"},                                             // 94
    {+0.89, "Solid Colors: Blue Violet [- - Brightness]"},                                      // 95
    {+0.91, "Solid Colors: Violet [- - Brightness]"},                                           // 96
    {+0.93, "Solid Colors: White [- - Brightness]"},                                            // 97
    {+0.95, "Solid Colors: Gray [- - Brightness]"},                                             // 98
    {+0.97, "Solid Colors: Dark Gray [- - Brightness]"},                                        // 99
    {+0.99, "Solid Colors: Black [- - Brightness]"},                                            // 100
};

bool InfrastructureSubsystem::NoteSensor() noexcept
{
    return m_notesensor->Get();
}

uint32_t InfrastructureSubsystem::GetLEDPatternCount() noexcept
{
    return LEDPatterns.size();
}

std::string_view InfrastructureSubsystem::GetLEDPatternDescription(uint32_t pattern) noexcept
{
    if (pattern >= LEDPatterns.size())
    {
        return "Bad Index!";
    }

    return LEDPatterns[pattern].second;
}

void InfrastructureSubsystem::SetLEDPattern(uint32_t pattern) noexcept
{
    if (pattern >= LEDPatterns.size())
    {
        return;
    }

    leds_->Set(LEDPatterns[pattern].first);
}

void InfrastructureSubsystem::SetNumberLights(bool on) noexcept
{
    pdh_->SetSwitchableChannel(on);
}

void InfrastructureSubsystem::Disable() noexcept
{
}

//  https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf. - LEDS
