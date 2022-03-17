#include "subsystems/FeederSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

FeederSubsystem::FeederSubsystem() noexcept
{
    m_intakeMotor = SparkMaxFactory::CreateSparkMax("Intake", 9, false);
    m_elevatorMotor = SparkMaxFactory::CreateSparkMax("Elevator", 10, true);
    m_feederMotor = SparkMaxFactory::CreateSparkMax("Feeder", 11, true);
    m_shooterMotor = SparkMaxFactory::CreateSparkMax("Shooter", 13, false);
    m_backspinMotor = SparkMaxFactory::CreateSparkMax("Backspin", 14, false);
    m_climberMotor = SparkMaxFactory::CreateSparkMax("Climber", 12, false);

    m_intakeRelease = std::make_unique<frc::DoubleSolenoid>(1, frc::PneumaticsModuleType::REVPH, 0, 1);
    m_intakeRaise = std::make_unique<frc::DoubleSolenoid>(1, frc::PneumaticsModuleType::REVPH, 2, 3);

    Pneumatics();
}

void FeederSubsystem::Periodic() noexcept
{
    m_intakeMotor->Periodic();
    m_elevatorMotor->Periodic();
    m_feederMotor->Periodic();
    m_shooterMotor->Periodic();
    m_backspinMotor->Periodic();
    m_climberMotor->Periodic();
}

void FeederSubsystem::Set(double percent) noexcept
{
    m_intakeMotor->SetVoltage(percent * 12_V);
    m_elevatorMotor->SetVoltage(percent * 12_V);
    m_feederMotor->SetVoltage(percent * 12_V);
    m_shooterMotor->SetVoltage(percent * 12_V);
    m_backspinMotor->SetVoltage(percent * 12_V);
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

void FeederSubsystem::LowerIntake() noexcept
{
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kForward);
}

void FeederSubsystem::RaiseIntake() noexcept
{
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kReverse);
}
