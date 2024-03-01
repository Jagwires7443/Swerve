// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include "rev/CANSparkMax.h"
#include <frc/DigitalOutput.h>

class ClimberSubsystem : public frc2::SubsystemBase 
{
 public:
  ClimberSubsystem() noexcept;

  ClimberSubsystem(const ClimberSubsystem &) = delete;
  ClimberSubsystem &operator=(const ClimberSubsystem &) = delete;

  void StopClimber() noexcept;
  void SetClimberMotorVoltagePercent(const double percent) noexcept;
  void SolenoidUp() noexcept;
  void SolenoidDown() noexcept;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  rev::CANSparkMax m_ClimberMotor{climber::kClimberMotorCanID, rev::CANSparkMax::MotorType::kBrushed};
  frc::DigitalOutput SolenoidSwitch{7};
};
