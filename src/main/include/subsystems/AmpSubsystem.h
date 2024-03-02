// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <frc/Timer.h>
#include "rev/CANSparkMax.h"

class AmpSubsystem : public frc2::SubsystemBase {
 public:
  AmpSubsystem() noexcept;
  
  AmpSubsystem(const AmpSubsystem &) = delete;
  AmpSubsystem &operator=(const AmpSubsystem &) = delete;

  void SetAmpExtendMotorVoltagePercent(const double percent) noexcept;
  void SetAmpHolderMotorVoltagePercent(const double percent) noexcept;
  void StopAmpExtend() noexcept;
  void StopAmpHolder() noexcept;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  //void Periodic() override;

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
  rev::CANSparkMax m_AmpExtendMotor{amp::kAmpExtendMotorCanID, rev::CANSparkMax::MotorType::kBrushed};
  rev::CANSparkMax m_AmpHolderMotor{amp::kAmpHolderMotorCanID, rev::CANSparkMax::MotorType::kBrushed};

};
