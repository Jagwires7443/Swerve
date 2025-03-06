#include "subsystems/HookSubsystem.h"
//#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include <units/angle.h>
#include <units/torque.h>
#include <units/voltage.h>

#include <cmath>
//Im not sure what to do with PID yet
// we need to set an encoder limit for the climber
// We need only one set angle for the right position the hooks need to be to make it climb

HookSubsystem::HookSubsystem() noexcept
{
    HookMotorBase_ = SparkMaxFactory::CreateSparkMax("Hook", nonDrive::kShoulderMotorCanID, nonDrive::kShoulderMotorInverted);
    HookSensor_ = std::make_unique<HookSensor>(nonDrive::kShoulderEncoderPort, nonDrive::kShoulderAlignmentOffset);
    HookMotor_ = std::make_unique<SmartMotor<units::angle::degrees>>(*HookMotorBase_);

    HookMotor_->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint32_t{250}}, // ms
        {"kStatus2", uint32_t{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
        {"kIdleMode", uint32_t{1}}, // Brake mode
    });

    HookMotor_->ApplyConfig(false);

    HookPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        arm::kShoulderPositionP,
        0.0,
        0.0,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            arm::kShoulderPositionMaxVelocity,
            arm::kShoulderPositionMaxAcceleration}));
    HookPIDController_->DisableContinuousInput();
    HookPIDController_->SetTolerance(arm::kShoulderTolerance);

    Reset();
}
void HookSubsystem::Reset() noexcept
{
    const auto HookSensor = HookSensor_->GetAbsolutePosition();

    if (HookSensor)
    {
        HookAngle_ = HookSensor.value();
    }
    else
    {
        HookAngle_ = arm::shoulderPositiveStopLimit;
    }
    SetHookAngle(arm::ArmBeginning);
    HookPIDController_->Reset(HookAngle_);
}

void HookSubsystem::SetHookAngle(units::angle::degree_t angle) noexcept
{
    units::angle::degree_t rotatedAngle = angle;
    CommandedHookAngle_ = angle;
}

void HookSubsystem::Periodic() noexcept
{
    HookSensor_->Periodic();
    HookMotor_->Periodic();

    const auto HookSensor = HookSensor_->GetAbsolutePosition();

    if (!HookSensor)
    {
        HookMotor_->Stop();
        status_ = false;
        printf("no sensor\n");
        return;
    }

    HookAngle_ = HookSensor.value();
    status_ = true;
    HookAngleGyro_.Set(HookAngle_.value());
    double shoulder = HookPIDController_->Calculate(HookAngle_);
    HookPower_ = shoulder;
    HookMotor_->SetVoltage(shoulder * 12.0_V);
}

bool HookSubsystem::InPosition() noexcept
{
    if (!HookPIDController_->AtGoal())
    {
        return false;
    }
    units::angle::degree_t shoulderError = CommandedHookAngle_ - HookAngle_;

    if (shoulderError < 0.0_deg)
    {
        shoulderError *= -1.0;
    }
    return (shoulderError < arm::kShoulderTolerance);

}
void HookSubsystem::ArmAtAmp() noexcept
{
    SetHookAngle(+96.0_deg);
}
void HookSubsystem::ArmAtSpeaker() noexcept
{
    SetHookAngle(+36.0_deg);
}

void HookSubsystem::ShoulderUp() noexcept
{
    SetHookAngle(+7.0_deg);
}
void HookSubsystem::ShoulderDown() noexcept
{
    SetHookAngle(-7.0_deg);
}

void HookSubsystem::ArmAtPickup() noexcept
{
    SetHookAngle(+11.0_deg);
}
void HookSubsystem::Stow() noexcept
{
    SetHookAngle(+17.0_deg); // originally 45
}

void FlipperSubsystem::SpeakerFar() noexcept
{
    SetFlipperAngle(+53.0_deg);
}


class FlipperSubsystem {

public:
    void SpeakerFar() noexcept;
    void BurnConfig() noexcept;

private:
    void SetFlipperAngle(units::angle::degree_t angle) noexcept;
    std::unique_ptr<SmartMotor<units::angle::degrees>> FlipperMotor_;
    std::unique_ptr<frc::ProfiledPIDController<units::angle::degrees>> FlipperPIDController_;
};

