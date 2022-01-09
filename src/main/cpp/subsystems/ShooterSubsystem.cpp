#include "subsystems/ShooterSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

ShooterSubsystem::ShooterSubsystem() noexcept
{
    DoSafeShooterMotors("ctor", [&]() -> void
                        {
        m_shooterOneMotor = std::make_unique<rev::CANSparkMax>(
            nonDrive::kShooterOneCanID,
            rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        m_shooterTwoMotor = std::make_unique<rev::CANSparkMax>(
            nonDrive::kShooterTwoCanID,
            rev::CANSparkMaxLowLevel::MotorType::kBrushless);

        if (!m_shooterOneMotor || !m_shooterTwoMotor)
        {
            throw std::runtime_error("m_feederMotor");
        }

        m_shooterOneMotor->SetInverted(false);
        m_shooterTwoMotor->SetInverted(true); });
}

void ShooterSubsystem::DoSafeShooterMotors(const char *const what, std::function<void()> work) noexcept
{
    try
    {
        work();
    }
    catch (const std::exception &e)
    {
        m_shooterOneMotor = nullptr;
        m_shooterOneEncoder = nullptr;
        m_shooterOnePID = nullptr;

        m_shooterTwoMotor = nullptr;
        m_shooterTwoEncoder = nullptr;
        m_shooterTwoPID = nullptr;

        std::printf("Shooter Motors %s exception: %s.\n", what, e.what());
    }
    catch (...)
    {
        m_shooterOneMotor = nullptr;
        m_shooterOneEncoder = nullptr;
        m_shooterOnePID = nullptr;

        m_shooterTwoMotor = nullptr;
        m_shooterTwoEncoder = nullptr;
        m_shooterTwoPID = nullptr;

        std::printf("Shooter Motors %s unknown exception.\n", what);
    }
}

void ShooterSubsystem::Set(double percent) noexcept
{
    DoSafeShooterMotors("Set()", [&]() -> void
                        {
        if (!m_shooterOneMotor || !m_shooterTwoMotor)
        {
            return;
        }

        m_shooterOneMotor->SetVoltage(percent * 12_V);
        m_shooterTwoMotor->SetVoltage(percent * 12_V); });
}
