#pragma once

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"
#include "infrastructure/PWMAngleSensor.h"
#include "infrastructure/ShuffleboardWidgets.h"

#include "infrastructure/SparkMax.h"
#include <frc2/command/button/CommandXboxController.h>


#include <units/angle.h>
#include <units/length.h>

#include <memory>
#include <string>

class FlipperSubsystem : public frc2::SubsystemBase {

public:

  void SetAngle(units::degree_t targetAngle);
  void Stop();
  units::degree_t GetCurrentAngle();
 
  FlipperSubsystem() noexcept;

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
    return flipperAngle_;
  }
  void Reset() noexcept;

  void SetShoulder(double percent) noexcept
  {
    flipperControlUI_ = percent;
  }

  void SetFlipperAngle(units::angle::degree_t angle) noexcept;
  void SetAngles(units::angle::degree_t shoulderAngle) noexcept
  {
    SetFlipperAngle(shoulderAngle);
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

  std::unique_ptr<AngleSensor> FlipperSensor_;
  std::unique_ptr<SmartMotorBase> FlipperMotorBase_;
  std::unique_ptr<SmartMotor<units::angle::degrees>> FlipperMotor_;
  std::unique_ptr<frc::ProfiledPIDController<units::angle::degrees>> FlipperPIDController_;
  std::unique_ptr<TuningPID> FlipperPIDControllerUI_;

  units::angle::degree_t commandedFlipperAngle_{0.0};
  units::angle::degree_t flipperAngle_{0.0_deg};

  double flipperControlUI_{0.0};
  bool flipperResetUI_{false};

  frc::ComplexWidget *flipperPIDUI_{nullptr};

  bool status_{false};
  bool print_{true};
  bool test_{false};

  HeadingGyro flipperAngleGyro_;


  double flipperPower_{0.0};
  double flipperFeedforward_{0.0};

  std::string notes_;
  frc::ComplexWidget *flipperAngleUI_{nullptr};
  frc::SimpleWidget *flipperErrorUI_{nullptr};
  frc::SimpleWidget *flipperPowerUI_{nullptr};
  frc::SimpleWidget *flipperFeedforwardUI_{nullptr};

  static constexpr units::degree_t kStowed = 0_deg;
  static constexpr units::degree_t kPickup = 30_deg;
  static constexpr units::degree_t kScoreLow = 60_deg;
   static constexpr units::degree_t kScoreHigh = 120_deg;
   //the units and degrees above will be changed for 2025
};

