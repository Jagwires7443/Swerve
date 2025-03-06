//  hooks/climbers subsystems
//references 2024 robot arm code

#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>

#include "Constants.h"
#include "infrastructure/PWMAngleSensor.h"
#include "infrastructure/ShuffleboardWidgets.h"
#include "infrastructure/SparkMax.h"

#include <units/angle.h>
#include <units/length.h>

#include <memory>
#include <string>

class HookSubsystem : public frc2::SubsystemBase
{
public:
  HookSubsystem() noexcept;

  void Periodic() noexcept override;

  void TestInit() noexcept;
  void TestExit() noexcept;
  void TestPeriodic() noexcept;
  void DisabledInit() noexcept;
  void DisabledExit() noexcept;

  bool Status() noexcept
  {
    return status_;
  }

  units::angle::degree_t GetAngle() noexcept 
  {
    return HookAngle_; 
  }

  void Reset() noexcept;

  void SetHook(double percent) noexcept //not sure what to change "SetHook" to yet
  {
    HookControlUI_ = percent; //HookControlUI needs to be renamed
  }

  void SetHookAngle(units::angle::degree_t angle) noexcept;
  
  void SetAngles(units::angle::degree_t HookAngle) noexcept
  {
    SetHookAngle(HookAngle);
  }

  bool InPosition() noexcept;

  void BurnConfig() noexcept;

  void ClearFaults() noexcept;

  void ArmAtAmp() noexcept;
  void ArmAtSpeaker() noexcept;
  void ShoulderUp() noexcept;
  void ShoulderDown() noexcept;
  void ArmAtPickup() noexcept;
  void Stow() noexcept;
  void SpeakerFar() noexcept;

  void HookUp() noexcept;
  void HookDown() noexcept;

private:
  std::unique_ptr<AngleSensor> HookSensor_;
  std::unique_ptr<SmartMotorBase> HookMotorBase_;
  std::unique_ptr<SmartMotor<units::angle::degrees>> HookMotor_;
  std::unique_ptr<frc::ProfiledPIDController<units::angle::degrees>> HookPIDController_;
  std::unique_ptr<TuningPID> HookPIDControllerUI_;

  units::angle::degree_t CommandedHookAngle_{0.0};
  units::angle::degree_t HookAngle_{0.0_deg};

  double HookControlUI_{0.0};
  bool HookResetUI_{false};

  frc::ComplexWidget *HookPIDUI_{nullptr};

  bool status_{false};
  bool print_{true};
  bool test_{false};

  HeadingGyro HookAngleGyro_;

  double HookPower_{0.0};
  double HookFeedforward_{0.0};

  std::string notes_;
  frc::ComplexWidget *HookAngleUI_{nullptr};
  frc::SimpleWidget *HookErrorUI_{nullptr};
  frc::SimpleWidget *HookPowerUI_{nullptr};
  frc::SimpleWidget *HookFeedforwardUI_{nullptr};

};
