#include "subsystems/FeederSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

FeederSubsystem::FeederSubsystem() noexcept
{
    const SmartMotorBase::ConfigMap config = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{0}},
        {"kRampRate", double{0.1}},
        {"kSmartCurrentStallLimit", uint{30}}, // Amps
        {"kSmartCurrentFreeLimit", uint{10}},  // Amps
    };

    m_intakeMotor = SparkMaxFactory::CreateSparkMax("Intake", 9, false);
    m_elevatorMotor = SparkMaxFactory::CreateSparkMax("Elevator", 10, true);
    m_feederMotor = SparkMaxFactory::CreateSparkMax("Feeder", 11, true);
    m_climberMotor = SparkMaxFactory::CreateSparkMax("Climber", 12, false);

    m_intakeRaise = std::make_unique<frc::DoubleSolenoid>(1, frc::PneumaticsModuleType::REVPH, 0, 1);
    m_intakeRelease = std::make_unique<frc::DoubleSolenoid>(1, frc::PneumaticsModuleType::REVPH, 2, 3);

    m_intakeMotor->SetConfig(config);
    m_elevatorMotor->SetConfig(config);
    m_feederMotor->SetConfig(config);

    m_intakeMotor->ApplyConfig(false);
    m_elevatorMotor->ApplyConfig(false);
    m_feederMotor->ApplyConfig(false);

    Pneumatics();
}

void FeederSubsystem::Periodic() noexcept
{
    m_intakeMotor->Periodic();
    m_elevatorMotor->Periodic();
    m_feederMotor->Periodic();
    m_climberMotor->Periodic();
}

void FeederSubsystem::Set(double percent) noexcept
{
    m_elevatorMotor->SetVoltage(percent * 12_V);
    m_intakeMotor->SetVoltage(percent * 12_V);

    if (percent < 0.0)
    {
        m_feederMotor->SetVoltage(percent * 12_V);
    }
    else
    {
        m_feederMotor->SetVoltage(0.0_V);
    }
}

void FeederSubsystem::Hold() noexcept
{
    m_feederMotor->Stop();
}

void FeederSubsystem::Fire() noexcept
{
    m_feederMotor->SetVoltage(12_V);
}

void FeederSubsystem::Pneumatics() noexcept
{
    m_intakeRelease->Set(frc::DoubleSolenoid::Value::kOff);
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kOff);
}

void FeederSubsystem::LockIntake() noexcept
{
    m_intakeRelease->Set(frc::DoubleSolenoid::Value::kForward);
}

void FeederSubsystem::DropIntake() noexcept
{
    m_intakeRelease->Set(frc::DoubleSolenoid::Value::kReverse);
}

void FeederSubsystem::RaiseIntake() noexcept
{
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kForward);
}

void FeederSubsystem::LowerIntake() noexcept
{
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kReverse);
}

void FeederSubsystem::Raise() noexcept
{
}

void FeederSubsystem::Lower() noexcept
{
}
