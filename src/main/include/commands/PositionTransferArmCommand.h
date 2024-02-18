// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <units/angle.h>
#include <frc/Timer.h>

#include "subsystems/TransferArmSubsystem.h"

#include <memory>

// Expose turning maximum velocity and acceleration.
class PositionTransferArm
    : public frc2::CommandHelper<frc2::Command, PositionTransferArm>
{
public:
    explicit PositionTransferArm(TransferArmSubsystem *transferArmSubsystem, units::degree_t position) noexcept
        : transferArmSubsystem{transferArmSubsystem}, position{position}
    {
        SetName("PositionTransferArm");
        AddRequirements(transferArmSubsystem);
    }

    void Initialize() noexcept override;
    void Execute() noexcept override;
    void End(bool interrupted) noexcept override;
    bool IsFinished() noexcept override { return finished; }

    static frc2::CommandPtr PositionTransferArmCommandFactory(TransferArmSubsystem *transferArmSubsystem, units::degree_t position) noexcept
    {
        return frc2::CommandPtr{std::make_unique<PositionTransferArm>(transferArmSubsystem, position)};
    }

private:
    TransferArmSubsystem *transferArmSubsystem{nullptr};
    units::degree_t position;
    frc::Timer timer{};
    bool finished{false};
};
