// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/PIDCommand.h>
#include <units/angle.h>
#include <frc/Timer.h>
#include "subsystems/TransferArmSubsystem.h"
#include "Constants.h"

/**
 * A command that will turn the transfer arm to the specified angle.
 */
class PIDPositionTransferArm : public frc2::CommandHelper<frc2::PIDCommand, PIDPositionTransferArm> {
 public:
  /**
   * Turns the transfer arm to robot to the specified angle.
   *
   * @param targetAngleDegrees          The angle to turn to
   * @param transferArmSubsystem        The transferArmSubsystem subsystem to use
   */
  PIDPositionTransferArm(units::turn_t targetPosition, TransferArmSubsystem* transferArmSubsystem);

  bool IsFinished() override;
};
