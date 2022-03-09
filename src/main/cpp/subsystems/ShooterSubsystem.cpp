#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

ShooterSubsystem::ShooterSubsystem() noexcept
{
    m_shooterMotor = SparkMaxFactory::CreateSparkMax("Shooter", 1, false);
}

void ShooterSubsystem::Periodic() noexcept
{
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    uint deltaTime = FPGATime - FPGATime_;

    if (deltaTime > 30000000) // 30-second tick
    {
        FPGATime_ = FPGATime;

        if (sequence_ == 0)
        {
            printf("**** Check.\n");
            m_shooterMotor->CheckConfig();

            ++sequence_;
        }
        else
        {
            printf("**** Apply.\n");
            m_shooterMotor->ApplyConfig(false);

            sequence_ = 0;
        }
    }

    m_shooterMotor->Periodic();
}

void ShooterSubsystem::Set(double percent) noexcept
{
    m_shooterMotor->SetVoltage(percent * 12_V);
}
