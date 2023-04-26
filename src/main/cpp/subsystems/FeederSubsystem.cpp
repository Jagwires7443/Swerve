#include "subsystems/FeederSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

FeederSubsystem::FeederSubsystem() noexcept
{
    // NEO 550.
    const SmartMotorBase::ConfigMap config = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{0}},
        {"kRampRate", double{0.1}},
        {"kSmartCurrentStallLimit", uint{25}}, // Amps
        {"kSmartCurrentFreeLimit", uint{10}},  // Amps
    };

    const SmartMotorBase::ConfigMap climberConfig = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kRampRate", double{0.5}},
        {"kSmartCurrentStallLimit", uint{40}}, // Amps
        // {"kSoftLimitFwd", double{0.0}},
        // {"kSoftLimitRev", double{0.0}},
    };

    const SmartMotorBase::ConfigMap moreTorque = {
        {"kSmartCurrentStallLimit", uint{30}}, // Amps
    };

    m_intakeMotorBase = SparkMaxFactory::CreateSparkMax("Intake", 9, false);
    m_elevatorMotorBase = SparkMaxFactory::CreateSparkMax("Elevator", 10, true);
    m_feederMotorBase = SparkMaxFactory::CreateSparkMax("Feeder", 11, true);
    m_climberMotorBase = SparkMaxFactory::CreateSparkMax("Climber", 12, false);
    m_intakeMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_intakeMotorBase);
    m_elevatorMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_elevatorMotorBase);
    m_feederMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_feederMotorBase);
    m_climberMotor = std::make_unique<SmartMotor<units::angle::turns>>(*m_climberMotorBase);

    m_intakeRaise = std::make_unique<frc::DoubleSolenoid>(1, frc::PneumaticsModuleType::REVPH, 0, 1);
    m_intakeRelease = std::make_unique<frc::DoubleSolenoid>(1, frc::PneumaticsModuleType::REVPH, 13, 12);

    m_intakeMotor->SetConfig(config);
    m_elevatorMotor->SetConfig(config);
    m_feederMotor->SetConfig(config);
    m_climberMotor->SetConfig(climberConfig);
    m_elevatorMotor->AddConfig(moreTorque);

    m_intakeMotor->ApplyConfig(false);
    m_elevatorMotor->ApplyConfig(false);
    m_feederMotor->ApplyConfig(false);
    m_climberMotor->ApplyConfig(false);

    Default(0.0);
}

void FeederSubsystem::Periodic() noexcept
{
    m_intakeMotor->Periodic();
    m_elevatorMotor->Periodic();
    m_feederMotor->Periodic();
    m_climberMotor->Periodic();
}

bool FeederSubsystem::GetStatus() const noexcept
{
    return m_intakeMotor->GetStatus() ||
           m_elevatorMotor->GetStatus() ||
           m_feederMotor->GetStatus() ||
           m_climberMotor->GetStatus();
}

void FeederSubsystem::TestInit() noexcept {}
void FeederSubsystem::TestExit() noexcept {}

void FeederSubsystem::TestPeriodic() noexcept
{
    const std::chrono::time_point now = std::chrono::steady_clock::now();
    if (now >= m_verifyMotorControllersWhen)
    {
        using namespace std::chrono_literals;

        m_verifyMotorControllersWhen = now + 15s;

        m_intakeMotor->CheckConfig();
        m_elevatorMotor->CheckConfig();
        m_feederMotor->CheckConfig();
        m_climberMotor->CheckConfig();
    }
}

void FeederSubsystem::Default(const double percent) noexcept
{
    m_intakeMotor->SetVoltage(percent * 12_V);
    m_elevatorMotor->SetVoltage(percent * 12_V);

    m_climberMotor->Stop();

    m_intakeRelease->Set(frc::DoubleSolenoid::Value::kOff);
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kOff);
}

void FeederSubsystem::NoFeed() noexcept
{
    m_feederMotor->Stop();
}

void FeederSubsystem::Eject() noexcept
{
    m_intakeMotor->SetVoltage(-12.0_V);
    m_elevatorMotor->SetVoltage(-12.0_V);
    m_feederMotor->SetVoltage(-12.0_V);

    m_climberMotor->Stop();

    m_intakeRelease->Set(frc::DoubleSolenoid::Value::kOff);
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kOff);
}

void FeederSubsystem::Fire() noexcept
{
    m_intakeMotor->Stop();
    m_elevatorMotor->Stop();

    m_feederMotor->SetVoltage(12_V);

    m_climberMotor->Stop();

    m_intakeRelease->Set(frc::DoubleSolenoid::Value::kOff);
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kOff);
}

void FeederSubsystem::Raise() noexcept
{
    m_intakeMotor->Stop();
    m_elevatorMotor->Stop();
    m_feederMotor->Stop();

    m_climberMotor->SetVoltage(-10.0_V);

    m_intakeRelease->Set(frc::DoubleSolenoid::Value::kOff);
    m_intakeRaise->Set(frc::DoubleSolenoid::Value::kOff);
}

void FeederSubsystem::Lower() noexcept
{
    m_intakeMotor->Stop();
    m_elevatorMotor->Stop();
    m_feederMotor->Stop();

    m_climberMotor->SetVoltage(10.0_V);

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

void FeederSubsystem::BurnConfig() noexcept
{
    m_intakeMotor->ApplyConfig(true);
    m_elevatorMotor->ApplyConfig(true);
    m_feederMotor->ApplyConfig(true);
    m_climberMotor->ApplyConfig(true);
}

void FeederSubsystem::ClearFaults() noexcept
{
    m_intakeMotor->ClearFaults();
    m_elevatorMotor->ClearFaults();
    m_feederMotor->ClearFaults();
    m_climberMotor->ClearFaults();
}
