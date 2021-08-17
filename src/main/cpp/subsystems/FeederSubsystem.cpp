#include "subsystems/FeederSubsystem.h"

#include "Constants.h"

FeederSubsystem::FeederSubsystem() noexcept
{
    DoSafeFeederMotors("ctor", [&]() -> void {
        m_feederOneMotor = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>(nonDrive::kFeederOneCanID);
        m_feederOneMotor = std::make_unique<ctre::phoenix::motorcontrol::can::WPI_VictorSPX>(nonDrive::kFeederTwoCanID);

        if (!m_feederOneMotor || !m_feederTwoMotor)
        {
            throw std::runtime_error("m_feederMotor");
        }

        m_feederOneMotor->SetInverted(true);
        m_feederTwoMotor->SetInverted(false);
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

void FeederSubsystem::Set(double percent) noexcept {}
