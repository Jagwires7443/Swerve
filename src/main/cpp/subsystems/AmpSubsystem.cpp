// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/AmpSubsystem.h"
#include  <units/voltage.h>

AmpSubsystem::AmpSubsystem() noexcept
{
    //Adjust the inverted status of the motors
    //m_AmpHolderMotor.SetInverted(amp::kAmpHolderMotorIsInverted);
    m_AmpExtendMotor.SetInverted(amp::kAmpExtendMotorIsInverted);
    StopAmpExtend();
    StopAmpHolder();
}

void AmpSubsystem::StopAmpExtend() noexcept
{
    m_AmpExtendMotor.StopMotor();
}

void AmpSubsystem::StopAmpHolder() noexcept
{
    //m_AmpHolderMotor.StopMotor();
}

void AmpSubsystem::SetAmpExtendMotorVoltagePercent(const double percent) noexcept
{
    m_AmpExtendMotor.SetVoltage(percent * 12.00_V);
}

void AmpSubsystem::SetAmpHolderMotorVoltagePercent(const double percent) noexcept
{
    //m_AmpHolderMotor.SetVoltage(percent * 12.00_V);
}

