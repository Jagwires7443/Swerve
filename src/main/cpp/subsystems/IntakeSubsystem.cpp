#include "subsystems/IntakeSubsystem.h"
#include <units/voltage.h>

IntakeSubsystem::IntakeSubsystem() noexcept
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

    // neos
    const SmartMotorBase::ConfigMap neoConfig = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kIdleMode", uint{1}},
        {"kRampRate", double{0.5}},
        {"kSmartCurrentStallLimit", uint{40}}, // Amps
        // {"kSoftLimitFwd", double{0.0}},
        // {"kSoftLimitRev", double{0.0}},
    };

    const SmartMotorBase::ConfigMap intakeConfig = {
        {"kSmartCurrentStallLimit", uint{1}}, // Amps
    };

    const SmartMotorBase::ConfigMap flexConfig = {
        {"kStatus1", uint{250}},
        {"kStatus2", uint{250}},
        {"kRampRate", double{0.5}},
        {"kSmartCurrentStallLimit", uint{60}}, // Amps
        // {"kSoftLimitFwd", double{0.0}},
        // {"kSoftLimitRev", double{0.0}},
    };

    const SmartMotorBase::ConfigMap climberConfig = {
        {"kIdleMode", uint32_t{1}}, // Brake.
    };

    // shoulderMotorBase_ = SparkMaxFactory::CreateSparkMax("Shoulder", nonDrive::kShoulderMotorCanID, nonDrive::kShoulderMotorInverted);
    m_intakeMotor = SparkMaxFactory::CreateSparkMax("Intake", nonDrive::kIntakeMotorCanID, nonDrive::kIntakeMotorInverted);
    m_triggerMotor = SparkMaxFactory::CreateSparkMax("Trigger", nonDrive::kTriggerMotorCanID, nonDrive::kTriggerMotorInverted);
    m_shooter1Motor = SparkMaxFactory::CreateSparkFlex("Shooter1", nonDrive::kShooter1MotorCanID, nonDrive::kShooter1MotorInverted); // invert all of thes
    m_shooter2Motor = SparkMaxFactory::CreateSparkFlex("Shooter2", nonDrive::kShooter2MotorCanID, nonDrive::kShooter2MotorInverted);
    m_climber1Motor = SparkMaxFactory::CreateSparkMax("Climber1", nonDrive::kClimber1MotorCanID, nonDrive::kClimber1MotorInverted);
    m_climber2Motor = SparkMaxFactory::CreateSparkMax("Climber2", nonDrive::kClimber2MotorCanID, nonDrive::kClimber2MotorInverted);

    m_intakeMotor->SetConfig(config);
    m_intakeMotor->AddConfig(intakeConfig);
    m_triggerMotor->SetConfig(config);
    m_shooter1Motor->SetConfig(flexConfig);
    m_shooter2Motor->SetConfig(flexConfig);
    m_climber1Motor->SetConfig(neoConfig);
    m_climber1Motor->AddConfig(climberConfig);
    m_climber2Motor->SetConfig(neoConfig);
    m_climber2Motor->AddConfig(climberConfig);

    m_intakeMotor->ApplyConfig(false);
    m_triggerMotor->ApplyConfig(false);
    m_shooter1Motor->ApplyConfig(false);
    m_shooter2Motor->ApplyConfig(false);
    m_climber1Motor->ApplyConfig(false);
    m_climber2Motor->ApplyConfig(false);

    Default(0.0);
}

void IntakeSubsystem::Periodic() noexcept
{
    m_intakeMotor->Periodic();
    m_triggerMotor->Periodic();
    m_shooter1Motor->Periodic();
    m_shooter2Motor->Periodic();
    m_climber1Motor->Periodic();
    m_climber2Motor->Periodic();
}

void IntakeSubsystem::TestInit() noexcept {}
void IntakeSubsystem::TestExit() noexcept {}
void IntakeSubsystem::TestPeriodic() noexcept {}
void IntakeSubsystem::Default(const double percent) noexcept
{
    /*
     m_intakeMotor->SetVoltage(12.0_V);
     m_triggerMotor->SetVoltage(12.0_V);
     */
    m_intakeMotor->SetVoltage(4.0_V); // set power a lot lower
                                      // m_triggerMotor->Stop();

    //// m_shooter1Motor->Stop();
    // m_shooter2Motor->Stop();
    // m_climber1Motor->Stop();
    // m_climber2Motor->Stop();
}

// bottom
void IntakeSubsystem::RunIntake() noexcept
{
    m_intakeMotor->SetVoltage(10.0_V);

    m_triggerMotor->Stop();
    m_climber1Motor->Stop();
    m_climber2Motor->Stop();
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
}

void IntakeSubsystem::StopIntake() noexcept
{
    m_intakeMotor->Stop();
    // m_triggerMotor->Stop();
    /*m_climber1Motor->Stop();
    m_climber2Motor->Stop();
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
    */
}

void IntakeSubsystem::ReverseIntake() noexcept
{
    m_intakeMotor->SetVoltage(-4.0_V);
    // m_triggerMotor->SetVoltage(12.0_V);
    // m_shooter1Motor->SetVoltage(-2.0_V);
    // m_shooter2Motor->SetVoltage(-2.0_V);

    m_climber1Motor->Stop();
    m_climber2Motor->Stop();
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
}
// Top

void IntakeSubsystem::FullOotas() noexcept
{
    m_shooter1Motor->SetVoltage(12.0_V);
    m_shooter2Motor->SetVoltage(12.0_V);
}

void IntakeSubsystem::Ootas() noexcept
{
    m_shooter1Motor->SetVoltage(7.0_V);
    m_shooter2Motor->SetVoltage(7.0_V);

    // if (counter <= 80)
    //{
    //    m_triggerMotor->SetVoltage(12.0_V);}
}

void IntakeSubsystem::ReverseOotas() noexcept
{
    m_triggerMotor->SetVoltage(-12.0_V);
    m_shooter1Motor->SetVoltage(-6.0_V);
    m_shooter2Motor->SetVoltage(-6.0_V);
}

void IntakeSubsystem::StopOotas() noexcept
{
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
}

void IntakeSubsystem::StopOotasandBodyT() noexcept
{
    m_shooter1Motor->Stop();
    m_shooter2Motor->Stop();
    m_triggerMotor->Stop();
}

void IntakeSubsystem::RunTrigger() noexcept
{
    m_triggerMotor->SetVoltage(12.0_V);
}

void IntakeSubsystem::StopTrigger() noexcept
{
    m_triggerMotor->Stop();
}

void IntakeSubsystem::Climb1() noexcept
{
    m_climber1Motor->SetVoltage(6.0_V);

    m_intakeMotor->Stop();
}

void IntakeSubsystem::Climb2() noexcept
{

    m_climber2Motor->SetVoltage(6.0_V);

    m_intakeMotor->Stop();
}

void IntakeSubsystem::StopClimb1() noexcept
{
    m_climber1Motor->Stop();
}

void IntakeSubsystem::StopClimb2() noexcept
{
    m_climber2Motor->Stop();
}

void IntakeSubsystem::ReverseClimb1() noexcept
{
    m_climber1Motor->SetVoltage(-6.0_V);

    m_intakeMotor->Stop();
}

void IntakeSubsystem::ReverseClimb2() noexcept
{
    m_climber2Motor->SetVoltage(-6.0_V);

    m_intakeMotor->Stop();
}
