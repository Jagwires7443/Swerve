#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>

#include <units/angle.h>
#include <units/torque.h>
#include <units/voltage.h>

#include <cmath>

ArmSubsystem::ArmSubsystem() noexcept
{
    shoulderMotorBase_ = SparkMaxFactory::CreateSparkMax("Shoulder", nonDrive::kShoulderMotorCanID, nonDrive::kShoulderMotorInverted);
    shoulderSensor_ = std::make_unique<AngleSensor>(nonDrive::kShoulderEncoderPort, nonDrive::kShoulderAlignmentOffset);
    shoulderMotor_ = std::make_unique<SmartMotor<units::angle::degrees>>(*shoulderMotorBase_);

    shoulderMotor_->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint32_t{250}}, // ms
        {"kStatus2", uint32_t{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
        {"kIdleMode", uint32_t{1}}, // Brake mode
    });

    shoulderMotor_->ApplyConfig(false);

    shoulderPIDController_ = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        arm::kShoulderPositionP,
        0.0,
        0.0,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            arm::kShoulderPositionMaxVelocity,
            arm::kShoulderPositionMaxAcceleration}));
    shoulderPIDController_->DisableContinuousInput();
    shoulderPIDController_->SetTolerance(arm::kShoulderTolerance);

    Reset();
}
void ArmSubsystem::Reset() noexcept
{
    const auto shoulderSensor = shoulderSensor_->GetAbsolutePosition();

    if (shoulderSensor)
    {
        shoulderAngle_ = shoulderSensor.value();
    }
    else
    {
        shoulderAngle_ = arm::shoulderPositiveStopLimit;
    }
    /*
        units::angle::degree_t rotatedShoulderAngle = shoulderAngle_;
        while (rotatedShoulderAngle >= +180.0_deg)
        {
            rotatedShoulderAngle -= 360.0_deg;
        }
        while (rotatedShoulderAngle < -180.0_deg)
        {
            rotatedShoulderAngle += 360.0_deg;
        }
    */
    SetShoulderAngle(arm::ArmBeginning);
    shoulderPIDController_->Reset(shoulderAngle_);
}

void ArmSubsystem::SetShoulderAngle(units::angle::degree_t angle) noexcept
{

    units::angle::degree_t rotatedAngle = angle;
    /*
        while (rotatedAngle >= +180.0_deg)
        {
            rotatedAngle -= 360.0_deg;
        }
        while (rotatedAngle < -180.0_deg)
        {
            rotatedAngle += 360.0_deg;
        }
        */

    commandedShoulderAngle_ = angle;
    shoulderPIDController_->SetGoal(angle);
}

void ArmSubsystem::Periodic() noexcept
{
    shoulderSensor_->Periodic();
    shoulderMotor_->Periodic();

    const auto shoulderSensor = shoulderSensor_->GetAbsolutePosition();

    if (!shoulderSensor)
    {
        shoulderMotor_->Stop();

        status_ = false;
        printf("no sensor\n");
        return;
    }

    shoulderAngle_ = shoulderSensor.value();

    status_ = true;

    shoulderAngleGyro_.Set(shoulderAngle_.value());
    // units::angle::degree_t rotatedShoulderAngle = shoulderAngle_;

    double shoulder = shoulderPIDController_->Calculate(shoulderAngle_);

    shoulderPower_ = shoulder;

    if (test_) // im not using test rn
    {
        shoulder = shoulderControlUI_;
    }

    if (shoulder > +arm::shoulderMaxPower)
    {
        shoulder = +arm::shoulderMaxPower;
    }
    if (shoulder < -arm::shoulderMaxPower)
    {
        shoulder = -arm::shoulderMaxPower;
    }

    if (shoulder < 0.0 && shoulderAngle_ < arm::shoulderNegativeStopLimit)
    {
        notes_ += " Shoulder -STOP";
        shoulder = 0.0;
    }
    else if (shoulder < -arm::shoulderParkPower && shoulderAngle_ < arm::shoulderNegativeParkLimit)
    {
        notes_ += " Shoulder -Park";
        shoulder = -arm::shoulderParkPower;
    }
    else if (shoulder < -arm::shoulderSlowPower && shoulderAngle_ < arm::shoulderNegativeSlowLimit)
    {
        notes_ += " Shoulder -Slow";
        shoulder = -arm::shoulderSlowPower;
    }

    if (shoulder > 0.0 && arm::shoulderPositiveStopLimit <= shoulderAngle_)
    {
        notes_ += " Shoulder +STOP";
        shoulder = 0.0;
    }
    else if (shoulder > +arm::shoulderParkPower && arm::shoulderPositiveParkLimit <= shoulderAngle_)
    {
        notes_ += " Shoulder +Park";
        shoulder = +arm::shoulderParkPower;
    }
    else if (shoulder > +arm::shoulderSlowPower && arm::shoulderPositiveSlowLimit <= shoulderAngle_)
    {
        notes_ += " Shoulder +Slow";
        shoulder = +arm::shoulderSlowPower;
    }

    if (print_)
    {
        printf(
            "**** Arm Status: ShoulderAngle=%lf Shoulderfeederforward%%=%lf ShoulderPIDSetGoal=%lf ShoulderPIDGetGoal=%lf ShoulderGetPositionError=%lf SO=%lf %s\n",
            shoulderAngle_.value(),
            shoulderFeedforward_,
            shoulderPIDController_->GetSetpoint().position.value(), // ??
            shoulderPIDController_->GetGoal().position.value(),
            shoulderPIDController_->GetPositionError().value(), // ??
            shoulder,
            notes_.c_str());
    }

    shoulderMotor_->SetVoltage(shoulder * 12.0_V);
}

bool ArmSubsystem::InPosition() noexcept
{
    if (!shoulderPIDController_->AtGoal())
    {
        return false;
    }
    units::angle::degree_t shoulderError = commandedShoulderAngle_ - shoulderAngle_;

    if (shoulderError < 0.0_deg)
    {
        shoulderError *= -1.0;
    }
    return (shoulderError < arm::kShoulderTolerance);
}

void ArmSubsystem::TestInit() noexcept
{
    frc::ShuffleboardTab &shuffleboardShoulderTab = frc::Shuffleboard::GetTab("Shoulder");

    frc::ShuffleboardLayout &shuffleboardLayoutShoulderSensor =
        shuffleboardShoulderTab.GetLayout("Sensor",
                                          frc::BuiltInLayouts::kGrid)
            .WithPosition(0, 0)
            .WithSize(8, 13)
            .WithProperties(wpi::StringMap<nt::Value>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(3.0))});

    frc::ShuffleboardLayout &shuffleboardLayoutShoulderMotor =
        shuffleboardShoulderTab.GetLayout("Motor",
                                          frc::BuiltInLayouts::kGrid)
            .WithPosition(8, 0)
            .WithSize(20, 6)
            .WithProperties(wpi::StringMap<nt::Value>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(6.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(2.0))});

    shoulderSensor_->ShuffleboardCreate(
        shuffleboardLayoutShoulderSensor,
        [&]() -> std::pair<units::angle::degree_t, units::angle::degree_t>
        { return std::make_pair(commandedShoulderAngle_, shoulderMotor_->GetPosition()); });

    frc::ShuffleboardLayout &shuffleboardLayoutShoulderPIDSettings =
        shuffleboardShoulderTab.GetLayout("PID Settings",
                                          frc::BuiltInLayouts::kGrid)
            .WithPosition(8, 6)
            .WithSize(19, 7)
            .WithProperties(wpi::StringMap<nt::Value>{
                std::make_pair("Number of columns", nt::Value::MakeDouble(3.0)),
                std::make_pair("Number of rows", nt::Value::MakeDouble(1.0))});

    shoulderMotor_->ShuffleboardCreate(
        shuffleboardLayoutShoulderMotor,
        [&](double control) -> void
        { shoulderControlUI_ = control; },
        [&]() -> void
        { shoulderControlUI_ = 0.0; shoulderResetUI_ = true; });

    shoulderMotor_->SetIdleMode(SmartMotorBase::IdleMode::kCoast);
    shoulderMotor_->Stop();

    Reset();
    test_ = true;
}
void ArmSubsystem::TestExit() noexcept
{
    test_ = false;

    shoulderMotor_->SetIdleMode(SmartMotorBase::IdleMode::kBrake);
    shoulderMotor_->Stop();
    Reset();
}

void ArmSubsystem::TestPeriodic() noexcept
{
    std::optional<int> shoulderPosition = shoulderSensor_->GetAbsolutePositionWithoutAlignment();
    if (shoulderPosition.has_value() && shoulderResetUI_)
    {
        // Work out new alignment so position becomes zero.
        int alignmentOffset = -shoulderPosition.value();
        if (alignmentOffset == +2048)
        {
            alignmentOffset = -2048;
        }

        shoulderSensor_->SetAlignment(alignmentOffset);
    }
    shoulderResetUI_ = false;

    shoulderErrorUI_->GetEntry()->SetDouble(shoulderPIDController_->GetPositionError().value());
    shoulderPowerUI_->GetEntry()->SetDouble(shoulderPower_);
    shoulderFeedforwardUI_->GetEntry()->SetDouble(shoulderFeedforward_);

    Periodic();
}

void ArmSubsystem::DisabledInit() noexcept {}
void ArmSubsystem::DisabledExit() noexcept
{
    shoulderMotor_->Stop();
    Reset();
}
void ArmSubsystem::BurnConfig() noexcept
{
    shoulderMotor_->ApplyConfig(true);
}
void ArmSubsystem::ClearFaults() noexcept
{
    shoulderMotor_->ClearFaults();
}

void ArmSubsystem::ArmAtAmp() noexcept
{
    SetShoulderAngle(+96.0_deg);
}
void ArmSubsystem::ArmAtSpeaker() noexcept
{
    SetShoulderAngle(+36.0_deg);
}

void ArmSubsystem::ShoulderUp() noexcept
{
    SetShoulderAngle(+7.0_deg);
}
void ArmSubsystem::ShoulderDown() noexcept
{
    SetShoulderAngle(-7.0_deg);
}

void ArmSubsystem::ArmAtPickup() noexcept
{
    SetShoulderAngle(+11.0_deg);
}
void ArmSubsystem::Stow() noexcept
{
    SetShoulderAngle(+17.0_deg); // originally 45
}
void ArmSubsystem::SpeakerFar() noexcept
{
    SetShoulderAngle(+53.0_deg);
}
