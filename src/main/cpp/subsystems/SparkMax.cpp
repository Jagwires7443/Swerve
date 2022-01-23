#include "subsystems/SparkMax.h"

#include <rev/CANSparkMax.h>
#include <rev/RelativeEncoder.h>
#include <rev/SparkMaxPIDController.h>

namespace
{
    class SparkMax : public SmartMotorBase
    {
    public:
        SparkMax(const int canId, const int encoderCounts) noexcept;

        SparkMax(const SparkMax &) = delete;
        SparkMax &operator=(const SparkMax &) = delete;

        void ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                std::function<void(double)> control = nullptr,
                                std::function<void()> reset = nullptr) noexcept override;

        void ShuffleboardPeriodic() noexcept override;

        void SetConfig(const ConfigMap config) noexcept override;

        void AddConfig(const ConfigMap config) noexcept override;

        bool CheckConfig() noexcept override;

        void ApplyConfig(bool burn) noexcept override;

        void ConfigPeriodic() noexcept override;

        void ClearFaults() noexcept override;

        bool GetStatus() noexcept override;

        void SetIdleMode(const IdleMode mode) noexcept override;

        IdleMode GetIdleMode() noexcept override;

        void Stop() noexcept override;

        void Set(const double percent) noexcept override;

        double Get() noexcept override;

        void SetVoltage(const units::volt_t voltage) noexcept override;

        void SpecifyPosition(const double position) noexcept override;

        void SeekPosition(const double position) noexcept override;

        bool CheckPosition(const double tolerance) noexcept override;

        double GetPositionRaw() noexcept override;

        void SeekVelocity(const double velocity) noexcept override;

        bool CheckVelocity(const double tolerance) noexcept override;

        double GetVelocityRaw() noexcept override;

    private:
        const int canId_;
        const int encoderCounts_;

        std::unique_ptr<rev::CANSparkMax> motor_;
        std::unique_ptr<rev::RelativeEncoder> encoder_;
        std::unique_ptr<rev::SparkMaxPIDController> controller_;
    };
}

std::unique_ptr<SmartMotorBase> SparkMaxFactory::CreateSparkMax(const int canId, const int encoderCounts) noexcept
{
    return std::make_unique<SparkMax>(canId, encoderCounts);
}

SparkMax::SparkMax(const int canId, const int encoderCounts) noexcept : canId_{canId}, encoderCounts_{encoderCounts} {}

void SparkMax::ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                  std::function<void(double)> control,
                                  std::function<void()> reset) noexcept {}

void SparkMax::ShuffleboardPeriodic() noexcept {}

void SparkMax::SetConfig(const ConfigMap config) noexcept {}

void SparkMax::AddConfig(const ConfigMap config) noexcept {}

bool SparkMax::CheckConfig() noexcept { return false; }

void SparkMax::ApplyConfig(bool burn) noexcept {}

void SparkMax::ConfigPeriodic() noexcept {}

void SparkMax::ClearFaults() noexcept {}

bool SparkMax::GetStatus() noexcept { return false; }

void SparkMax::SetIdleMode(const SmartMotorBase::IdleMode mode) noexcept {}

SmartMotorBase::IdleMode SparkMax::GetIdleMode() noexcept { return SmartMotorBase::IdleMode::kCoast; }

void SparkMax::Stop() noexcept {}

void SparkMax::Set(const double percent) noexcept {}

double SparkMax::Get() noexcept { return 0.0; }

void SparkMax::SetVoltage(const units::volt_t voltage) noexcept {}

void SparkMax::SpecifyPosition(const double position) noexcept {}

void SparkMax::SeekPosition(const double position) noexcept {}

bool SparkMax::CheckPosition(const double tolerance) noexcept { return false; }

double SparkMax::GetPositionRaw() noexcept { return 0.0; }

void SparkMax::SeekVelocity(const double velocity) noexcept {}

bool SparkMax::CheckVelocity(const double tolerance) noexcept { return false; }

double SparkMax::GetVelocityRaw() noexcept { return 0.0; }
