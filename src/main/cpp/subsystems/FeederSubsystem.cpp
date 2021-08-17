#include "subsystems/FeederSubsystem.h"

#include "Constants.h"

#include <units/voltage.h>

FeederSubsystem::FeederSubsystem() noexcept
{
    DoSafeFeederMotors("ctor", [&]() -> void {
        m_feederOneMotor = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>(nonDrive::kFeederOneCanID);
        m_feederTwoMotor = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>(nonDrive::kFeederTwoCanID);

        if (!m_feederOneMotor || !m_feederTwoMotor)
        {
            throw std::runtime_error("m_feederMotor");
        }

        m_feederOneMotor->SetInverted(false);
        m_feederTwoMotor->SetInverted(true);
    });
}

void FeederSubsystem::DoSafeFeederMotors(const char *const what, std::function<void()> work) noexcept
{
    try
    {
        work();
    }
    catch (const std::exception &e)
    {
        m_feederOneMotor = nullptr;
        m_feederTwoMotor = nullptr;

        std::printf("Feeder Motors %s exception: %s.\n", what, e.what());
    }
    catch (...)
    {
        m_feederOneMotor = nullptr;
        m_feederTwoMotor = nullptr;

        std::printf("Feeder Motors %s unknown exception.\n", what);
    }
}

void FeederSubsystem::Set(double percent) noexcept
{
    DoSafeFeederMotors("Set()", [&]() -> void {
        if (!m_feederOneMotor || !m_feederTwoMotor)
        {
            return;
        }

        m_feederOneMotor->SetVoltage(percent * 12_V);
        m_feederTwoMotor->SetVoltage(percent * 12_V);
    });
}
