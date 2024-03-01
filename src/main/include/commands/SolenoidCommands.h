#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include "subsystems/SolenoidSubsystem.h"

class SolenoidCommands
{
public:
    explicit SolenoidCommands(SolenoidSubsystem *SolenoidSubsystem)
      : SolenoidSubsystem{SolenoidSubsystem}
  {
    AddRequirements(SolenoidSubsystem);
  }

    void Initialize() override;
    void Execute() override;
    void End(bool interrupted) override;
    bool IsFinished() override;

private:
    SolenoidCommands *SolenoidCommands{nullptr};
    frc::Timer timer{};
    bool finished{false};
    frc::DigitalInput SolenoidSwitch{7};
}

