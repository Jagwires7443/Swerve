#include "subsystems/FlipperSubsystem.h"

#include "Constants.h"
#include "infrastructure/SparkMax.h"
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <units/angle.h>
#include "subsystems/Infrastructure.h"
#include <units/torque.h>
#include <units/voltage.h>
#include <cmath>
#include <rev/SparkMax.h>
#include <frc/XboxController.h>
#include <frc/smartdashboard/SmartDashboard.h>

FlipperSubsystem::FlipperSubsystem() noexcept
{
    FlipperMotorBase_ = SparkMaxFactory::CreateSparkMax("Flipper", nonDrive::kShoulderMotorCanID, nonDrive::kShoulderMotorInverted);
    FlipperSensor_ = std::make_unique<FlipperSensor>(nonDrive::kShoulderEncoderPort, nonDrive::kShoulderAlignmentOffset);
    FlipperMotor_ = std::make_unique<SmartMotor<units::angle::degrees>>(*FlipperMotorBase_);

    FlipperMotor_->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint32_t{250}}, // ms
        {"kStatus2", uint32_t{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
        {"kIdleMode", uint32_t{1}}, // Brake mode
    });

    FlipperMotor_->ApplyConfig(false);

    FlipperPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        arm::kShoulderPositionP,
        0.0,
        0.0,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            arm::kShoulderPositionMaxVelocity,
            arm::kShoulderPositionMaxAcceleration}));
    FlipperPIDController_->DisableContinuousInput();
    FlipperPIDController_->SetTolerance(arm::kShoulderTolerance);

    Reset();
}
void FlipperSubsystem::Reset() noexcept
{
    const auto FlipperSensor = FlipperSensor_->GetAbsolutePosition();

    if (FlipperSensor)
    {
        flipperAngle_ = FlipperSensor.value();
    }
    else
    {
        flipperAngle_ = arm::shoulderPositiveStopLimit;
    }
    SetFlipperAngle(arm::ArmBeginning);
    FlipperPIDController_->Reset(flipperAngle_);
}

void FlipperSubsystem::SetFlipperAngle(units::angle::degree_t angle) noexcept
{
    units::angle::degree_t rotatedAngle = angle;
    commandedFlipperAngle_ = angle;
}

void FlipperSubsystem::Periodic() noexcept
{
    FlipperSensor_->Periodic();
    FlipperMotor_->Periodic();

    const auto FlipperSensor = FlipperSensor_->GetAbsolutePosition();

    if (!FlipperSensor)
    {
        FlipperMotor_->Stop();
        status_ = false;
        printf("no sensor\n");
        return;
    }

    flipperAngle_ = FlipperSensor.value();
    status_ = true;
    flipperAngleGyro_.Set(flipperAngle_.value());
    double shoulder = FlipperPIDController_->Calculate(flipperAngle_);
    flipperPower_ = shoulder;
    FlipperMotor_->SetVoltage(shoulder * 12.0_V);
}

bool FlipperSubsystem::InPosition() noexcept
{
    if (!FlipperPIDController_->AtGoal())
    {
        return false;
    }
    units::angle::degree_t shoulderError = commandedFlipperAngle_ - flipperAngle_;

    if (shoulderError < 0.0_deg)
    {
        shoulderError *= -1.0;
    }
    return (shoulderError < arm::kShoulderTolerance);
}
//When making the preset angles i believe ; probably only going to be 3-4 presets stow, two for coral placement, and maybe one for climb
void FlipperSubsystem::ArmAtAmp() noexcept
{
    SetFlipperAngle(+96.0_deg);
}
void FlipperSubsystem::ArmAtSpeaker() noexcept
{
    SetFlipperAngle(+36.0_deg);
}

void FlipperSubsystem::ShoulderUp() noexcept
{
    SetFlipperAngle(+7.0_deg);
}
void FlipperSubsystem::ShoulderDown() noexcept
{
    SetFlipperAngle(-7.0_deg);
}

void FlipperSubsystem::ArmAtPickup() noexcept
{
    SetFlipperAngle(+11.0_deg);
}
void FlipperSubsystem::Stow() noexcept
{
    SetFlipperAngle(+17.0_deg); // originally 45
}
void FlipperSubsystem::SpeakerFar() noexcept
{
    SetFlipperAngle(+53.0_deg);
}





 