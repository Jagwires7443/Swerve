#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>
#include <rev/CANSparkMax.h>

#include <memory>

class ShooterSubsystem : public frc2::SubsystemBase
{
public:
    // The only ctor of the FeederSubsystem class.
    ShooterSubsystem() noexcept;

    ShooterSubsystem(const ShooterSubsystem &) = delete;
    ShooterSubsystem &operator=(const ShooterSubsystem &) = delete;

    void Set(double percent) noexcept;

private:
    void DoSafeShooterMotors(const char *const what, std::function<void()> work) noexcept;

    std::unique_ptr<rev::CANSparkMax> m_shooterOneMotor;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> m_shooterOneEncoder;
    std::unique_ptr<rev::SparkMaxPIDController> m_shooterOnePID;

    std::unique_ptr<rev::CANSparkMax> m_shooterTwoMotor;
    std::unique_ptr<rev::SparkMaxRelativeEncoder> m_shooterTwoEncoder;
    std::unique_ptr<rev::SparkMaxPIDController> m_shooterTwoPID;
};
