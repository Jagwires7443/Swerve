// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ClimberSubsystem.h"

ClimberSubsystem::ClimberSubsystem() noexcept
{
    m_ClimberMotor.SetInverted(climber::kClimberMotorIsInverted);
}

void ClimberSubsystem::StopClimber() noexcept
{
    m_ClimberMotor.StopMotor();
}

void ClimberSubsystem::SetClimberMotorVoltagePercent(const double percent) noexcept
{
    m_ClimberMotor.SetVoltage(percent * 12.00_V);
}
