// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>

#include "Constants.h"
#include "infrastructure/PWMAngleSensor.h"
#include "infrastructure/ShuffleboardWidgets.h"
#include "infrastructure/SparkMax.h"

#include <units/angle.h>
#include <units/length.h>
#include <units/voltage.h>

#include <memory>
#include <string>

class IntakeSubsystem : public frc2::SubsystemBase
{
public:
  IntakeSubsystem() noexcept;

  void Periodic() noexcept override;

  void TestInit() noexcept;
  void TestExit() noexcept;
  void TestPeriodic() noexcept;
  void DisabledInit() noexcept;
  void DisabledExit() noexcept;

  //  bool NoteSensor() noexcept;

  void Default(const double percent) noexcept;

  void RunIntake() noexcept;
  void StopIntake() noexcept;
  void ReverseIntake() noexcept;

  void FullOotas() noexcept;    // half power
  void Ootas() noexcept;        // shootspeaker / run speakers
  void ReverseOotas() noexcept; // reverses trigger and ootaassssszzz
  void StopOotas() noexcept;
  void StopOotasandBodyT() noexcept;

  void RunTrigger() noexcept; // run trigger
  void StopTrigger() noexcept;

  void Climb1() noexcept;
  void Climb2() noexcept;
  void StopClimb1() noexcept;
  void StopClimb2() noexcept;
  void ReverseClimb1() noexcept;
  void ReverseClimb2() noexcept;

private:
  std::unique_ptr<SmartMotorBase> m_intakeMotor;
  std::unique_ptr<SmartMotorBase> m_shooter1Motor;
  std::unique_ptr<SmartMotorBase> m_shooter2Motor;
  std::unique_ptr<SmartMotorBase> m_triggerMotor;
  std::unique_ptr<SmartMotorBase> m_climber1Motor;
  std::unique_ptr<SmartMotorBase> m_climber2Motor;
  // std::unique_ptr<frc::DigitalInput> m_notesensor;

  // bool NoteSensor_{};
};