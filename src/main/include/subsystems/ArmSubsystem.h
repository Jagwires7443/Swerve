
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

class ArmSubsystem : public frc2::SubsystemBase
{
public:
  ArmSubsystem() noexcept;

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

  units::angle::degree_t GetShoulderAngle() noexcept
  {
    return shoulderAngle_;
  }
  void Reset() noexcept;

  void SetShoulder(double percent) noexcept
  {
    shoulderControlUI_ = percent;
  }

  void SetShoulderAngle(units::angle::degree_t angle) noexcept;
  void SetAngles(units::angle::degree_t shoulderAngle) noexcept
  {
    SetShoulderAngle(shoulderAngle);
  }

  bool InPosition() noexcept;

  void BurnConfig() noexcept;

  void ClearFaults() noexcept;

  void ArmAtAmp() noexcept;
  void ArmAtSpeaker() noexcept;
  void ArmAtPickup() noexcept; // added this cuz i didnt think we had it
  void Stow() noexcept;
  void SpeakerFar() noexcept;

  void ShoulderUp() noexcept;
  void ShoulderDown() noexcept;

private:
  std::unique_ptr<AngleSensor> shoulderSensor_;
  std::unique_ptr<SmartMotorBase> shoulderMotorBase_;
  std::unique_ptr<SmartMotor<units::angle::degrees>> shoulderMotor_;
  std::unique_ptr<frc::ProfiledPIDController<units::angle::degrees>> shoulderPIDController_;
  std::unique_ptr<TuningPID> shoulderPIDControllerUI_;

  units::angle::degree_t commandedShoulderAngle_{0.0};
  units::angle::degree_t shoulderAngle_{0.0_deg};

  double shoulderControlUI_{0.0};
  bool shoulderResetUI_{false};

  frc::ComplexWidget *shoulderPIDUI_{nullptr};

  bool status_{false};
  bool print_{true};
  bool test_{false};

  HeadingGyro shoulderAngleGyro_;

  double shoulderPower_{0.0};
  double shoulderFeedforward_{0.0};

  std::string notes_;
  frc::ComplexWidget *shoulderAngleUI_{nullptr};
  frc::SimpleWidget *shoulderErrorUI_{nullptr};
  frc::SimpleWidget *shoulderPowerUI_{nullptr};
  frc::SimpleWidget *shoulderFeedforwardUI_{nullptr};
};