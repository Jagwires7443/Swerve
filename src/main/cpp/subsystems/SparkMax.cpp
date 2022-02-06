#include "subsystems/SparkMax.h"

#include <bitset>
#include <cstddef>
#include <exception>
#include <functional>
#include <iomanip>
#include <ios>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/RelativeEncoder.h>
#include <rev/REVLibError.h>
#include <rev/SparkMaxPIDController.h>

#include <frc/shuffleboard/ShuffleboardWidget.h>

namespace
{
    class SparkMax : public SmartMotorBase
    {
    public:
        SparkMax(const std::string_view name, const int canId, const bool inverted, const int encoderCounts) noexcept;

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
        const std::string name_;
        const int canId_;
        const bool inverted_;
        const int encoderCounts_;

        std::unique_ptr<rev::CANSparkMax> motor_;
        std::unique_ptr<rev::RelativeEncoder> encoder_;
        std::unique_ptr<rev::SparkMaxPIDController> controller_;

        frc::SimpleWidget *temperatureUI_ = nullptr;
        frc::SimpleWidget *statusUI_ = nullptr;
        frc::SimpleWidget *faultsUI_ = nullptr;
        frc::SimpleWidget *stickyFaultsUI_ = nullptr;
        frc::SimpleWidget *voltageUI_ = nullptr;
        frc::SimpleWidget *currentUI_ = nullptr;
        frc::SimpleWidget *percentUI_ = nullptr;
        frc::SimpleWidget *speedUI_ = nullptr;
        frc::SimpleWidget *distanceUI_ = nullptr;
        frc::SimpleWidget *velocityUI_ = nullptr;
        frc::SimpleWidget *controlUI_ = nullptr;
        frc::SimpleWidget *resetUI_ = nullptr;

        double previousControl_ = 0.0;
        std::function<void(double)> controlF_ = nullptr;
        std::function<void()> resetF_ = nullptr;

        bool configGood_ = false;

        void DoSafely(const char *const what, std::function<void()> work) noexcept;
        bool VerifyConfig(const std::string_view key, const ConfigValue &value) noexcept;
        void ApplyConfig(const std::string_view key, const ConfigValue &value) noexcept;
    };

    std::string FirmwareInfo(const uint32_t firmwareVersion, const std::string &firmwareString) noexcept
    {
        const std::bitset<8> firmwareFieldBuildL((firmwareVersion & 0x000000ff) >> 0);
        const std::bitset<8> firmwareFieldBuildH((firmwareVersion & 0x0000ff00) >> 8);
        const std::bitset<8> firmwareFieldMinor((firmwareVersion & 0x00ff0000) >> 16);
        const std::bitset<8> firmwareFieldMajor((firmwareVersion & 0xff000000) >> 24);

        const std::bitset<16> firmwareFieldBuild((firmwareFieldBuildH.to_ulong() << 8) |
                                                 (firmwareFieldBuildL.to_ulong()));

        std::stringstream firmwareVersionHex;

        firmwareVersionHex << std::internal << std::setfill('0') << std::setw(10)
                           << std::hex << std::showbase << firmwareVersion;

        std::string firmwareInfo;

        firmwareInfo += " FW(";
        firmwareInfo += firmwareVersionHex.str();
        firmwareInfo += "/";
        firmwareInfo += std::to_string(firmwareFieldMajor.to_ulong());
        firmwareInfo += ".";
        firmwareInfo += std::to_string(firmwareFieldMinor.to_ulong());
        firmwareInfo += ".";
        firmwareInfo += std::to_string(firmwareFieldBuild.to_ulong());
        firmwareInfo += "/\"";
        firmwareInfo += firmwareString;
        firmwareInfo += "\")";

        return firmwareInfo;
    }

    std::string FaultInfo(const uint16_t faults)
    {
        const std::bitset<16> faultBits(faults);

        // This works fine, but isn't very readable on the display:
        // return faultBits.to_string('.', '*');

        // SPARK MAX Fault Codes with shorthand characters used here.
        // B kBrownout
        // C kOvercurrent
        // W kIWDTReset
        // M kMotorFault
        // S kSensorFault
        // L kStall
        // E kEEPROMCRC
        // > kCANTX
        // < kCANRX
        // R kHasReset
        // D kDRVFault
        // O kOtherFault
        // ) kSoftLimitFwd
        // ( kSoftLimitRev
        // ] kHardLimitFwd
        // [ kHardLimitRev
        const char letters[] = {'B', 'C', 'W', 'M', 'S', 'L', 'E', '>', '<', 'R', 'D', 'O', ')', '(', ']', '['};

        std::string faultInfo;

        for (std::size_t i = faultBits.size(); i > 0; --i)
        {
            faultInfo += std::string(1, faultBits[i - 1] ? letters[i - 1] : '.');
        }

        return faultInfo;
    }
}

std::unique_ptr<SmartMotorBase> SparkMaxFactory::CreateSparkMax(const std::string_view name, const int canId, const bool inverted, const int encoderCounts) noexcept
{
    return std::make_unique<SparkMax>(name, canId, inverted, encoderCounts);
}

SparkMax::SparkMax(const std::string_view name, const int canId, const bool inverted, const int encoderCounts) noexcept : name_{name}, canId_{canId}, inverted_{inverted}, encoderCounts_{encoderCounts}
{
    DoSafely("ctor", [&]() -> void
             {
        motor_ = std::make_unique<rev::CANSparkMax>(canId_, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        if (!motor_)
        {
            throw std::runtime_error("motor_");
        }

        // Does not get sent to the motor controller, done locally.
        motor_->SetInverted(inverted_);

        if (encoderCounts_ == 0)
        {
            encoder_ = std::make_unique<rev::SparkMaxRelativeEncoder>(motor_->GetEncoder());
        }
        else
        {
            encoder_ = std::make_unique<rev::SparkMaxAlternateEncoder>(motor_->GetAlternateEncoder(encoderCounts_));
        }
        if (!encoder_)
        {
            throw std::runtime_error("encoder_");
        }

        controller_ = std::make_unique<rev::SparkMaxPIDController>(motor_->GetPIDController());
        if (!controller_)
        {
            throw std::runtime_error("controller_");
        }

        if (controller_->SetFeedbackDevice(*encoder_) != rev::REVLibError::kOk)
        {
            throw std::runtime_error("SetFeedbackDevice()");
        } });

    ClearFaults();
}

void SparkMax::ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                  std::function<void(double)> control,
                                  std::function<void()> reset) noexcept
{
    temperatureUI_ = &container.Add("Temperature", 0.0)
                          .WithPosition(0, 0);
    statusUI_ = &container.Add("Status", false)
                     .WithPosition(0, 1)
                     .WithWidget(frc::BuiltInWidgets::kBooleanBox);
    faultsUI_ = &container.Add("Faults", "")
                     .WithPosition(1, 0);

    stickyFaultsUI_ = &container.Add("StickyFaults", "")
                           .WithPosition(1, 1);

    voltageUI_ = &container.Add("Voltage", 0.0)
                      .WithPosition(2, 0);

    currentUI_ = &container.Add("Current", 0.0)
                      .WithPosition(2, 1);

    percentUI_ = &container.Add("Percent (Actual)", 0.0)
                      .WithPosition(3, 0)
                      .WithWidget(frc::BuiltInWidgets::kNumberBar);

    speedUI_ = &container.Add("Speed (Commanded)", 0.0)
                    .WithPosition(3, 1)
                    .WithWidget(frc::BuiltInWidgets::kNumberBar);

    distanceUI_ = &container.Add("Distance (Rotations)", 0.0)
                       .WithPosition(4, 0);

    velocityUI_ = &container.Add("Velocity (Rot per sec)", 0.0)
                       .WithPosition(4, 1);

    controlUI_ = &container.Add("Control", 0.0)
                      .WithPosition(5, 0)
                      .WithWidget(frc::BuiltInWidgets::kNumberSlider);

    resetUI_ = &container.Add("Reset", false)
                    .WithPosition(5, 1)
                    .WithWidget(frc::BuiltInWidgets::kToggleButton);
}

void SparkMax::ShuffleboardPeriodic() noexcept
{
    // Read controls information from Shuffleboard and manage interactive UI.
    const bool reset = resetUI_->GetEntry().GetBoolean(false);
    double control = controlUI_->GetEntry().GetDouble(0.0);

    if (reset)
    {
        resetUI_->GetEntry().SetBoolean(false);
        controlUI_->GetEntry().SetDouble(0.0);
        control = 0.0;

        if (resetF_)
        {
            resetF_();
        }
    }

    if (control != previousControl_)
    {
        previousControl_ = control;

        if (controlF_)
        {
            controlF_(control);
        }
    }

    double temperature = 0.0;
    uint16_t faults = 0;
    uint16_t stickyFaults = 0;
    double voltage = 0.0;
    double current = 0.0;
    double percent = 0.0;
    double speed = 0.0;
    double distance = 0.0;
    double velocity = 0.0;

    // Obtain raw data from CANSparkMax and set the output.
    DoSafely("ShuffleboardPeriodic()", [&]() -> void
             {
        if (motor_)
        {
            temperature = motor_->GetMotorTemperature();
            faults = motor_->GetFaults();
            stickyFaults = motor_->GetStickyFaults();
            voltage = motor_->GetBusVoltage();
            current = motor_->GetOutputCurrent();
            percent = motor_->GetAppliedOutput(); // Actual setting
            speed = motor_->Get();                // Commanded setting
        }

// XXX
        if (motor_ && reset)
        {
            if (motor_->ClearFaults() != rev::REVLibError::kOk)
            {
                throw std::runtime_error("ClearFaults()");
            }
        }

        if (encoder_)
        {
            distance = encoder_->GetPosition();        // Rotations
            velocity = encoder_->GetVelocity() / 60.0; // Rotations per second

// XXX
            if (reset)
            {
                // Logic above ensures that `position` is now zero; reset the
                // turning motor controller encoder to reflect this.
                if (encoder_->SetPosition(0.0) != rev::REVLibError::kOk)
                {
                    throw std::runtime_error("SetPosition()");
                }
            }
        } });

    temperatureUI_->GetEntry().SetDouble(temperature);
    statusUI_->GetEntry().SetBoolean(motor_ && configGood_ && faults == 0);
    faultsUI_->GetEntry().SetString(FaultInfo(faults));
    stickyFaultsUI_->GetEntry().SetString(FaultInfo(stickyFaults));
    voltageUI_->GetEntry().SetDouble(voltage);
    currentUI_->GetEntry().SetDouble(current);
    percentUI_->GetEntry().SetDouble(percent);
    speedUI_->GetEntry().SetDouble(speed);
    distanceUI_->GetEntry().SetDouble(distance);
    velocityUI_->GetEntry().SetDouble(velocity);
}

void SparkMax::SetConfig(const ConfigMap config) noexcept {}

void SparkMax::AddConfig(const ConfigMap config) noexcept {}

bool SparkMax::CheckConfig() noexcept { return false; }

void SparkMax::ApplyConfig(bool burn) noexcept {}

void SparkMax::ConfigPeriodic() noexcept {}

void SparkMax::ClearFaults() noexcept
{
    DoSafely("ClearFaults()", [&]() -> void
             {
        if (motor_)
        {
            if (motor_->ClearFaults() != rev::REVLibError::kOk)
            {
                throw std::runtime_error("ClearFaults()");
            }
        } });
}

bool SparkMax::GetStatus() noexcept
{
    uint16_t faults = 0;

    DoSafely("GetStatus()", [&]() -> void
             {
                 if (motor_)
                 {
                     faults = motor_->GetFaults();
                 } });

    return motor_ && configGood_ && faults == 0;
}

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

void SparkMax::DoSafely(const char *const what, std::function<void()> work) noexcept
{
    try
    {
        work();
    }
    catch (const std::exception &e)
    {
        motor_ = nullptr;
        encoder_ = nullptr;
        controller_ = nullptr;

        std::printf("SparkMax[%i] %s %s exception: %s.\n", canId_, name_.c_str(), what, e.what());
    }
    catch (...)
    {
        motor_ = nullptr;
        encoder_ = nullptr;
        controller_ = nullptr;

        std::printf("SparkMax[%i] %s %s unknown exception.\n", canId_, name_.c_str(), what);
    }
}

// XXX
// using ConfigValue = std::variant<bool, uint, double>;
// using ConfigMap = std::map<std::string, ConfigValue>;

bool SparkMax::VerifyConfig(const std::string_view key, const ConfigValue &value) noexcept
{
    const bool altMode = (encoderCounts_ != 0);
    const auto kv = SparkMaxFactory::configDefaults.find(std::string(key));

    if (kv == SparkMaxFactory::configDefaults.end())
    {
        // XXX coding error -- warn!

        return false;
    }

    const ConfigValue &default_value = kv->second;
    std::optional<ConfigValue> actual_value;

    // In order to avoid having to switch on a string, obtain the index of the entry.
    const auto ndx = std::distance(kv, SparkMaxFactory::configDefaults.begin());

    DoSafely("Get config parameter", [&]() -> void
             {
        if (motor_ && encoder_ && controller_)
        {
    switch (ndx)
    {
    case 0: // Firmware Version
        actual_value = uint{motor_->GetFirmwareVersion()};
        break;
    case 1: // kIdleMode
        {
            const rev::CANSparkMax::IdleMode tmp = motor_->GetIdleMode();

            if (tmp == rev::CANSparkMax::IdleMode::kCoast)
            {
                actual_value = uint{0};
            } else if (tmp == rev::CANSparkMax::IdleMode::kBrake)
            {
                actual_value = uint{1};
            }
        }
        break;
    case 2: // kFollowerID
        {
            if (motor_->IsFollower())
            {
            }
            else
            {
                actual_value = uint{0};
            }
        }
        break;
    case 3: // kFollowerConfig
        {
            if (motor_->IsFollower())
            {
            }
            else
            {
                actual_value = uint{0};
            }
        }
        break;
    case 4: // kSoftLimitFwd
        actual_value = double{motor_->GetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward)};
        break;
    case 5: // kSoftLimitRev
        actual_value = double{motor_->GetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse)};
        break;
    case 6: // kRampRate
        actual_value = double{motor_->GetOpenLoopRampRate()};
        break;
    case 7: // kClosedLoopRampRate
        actual_value = double{motor_->GetClosedLoopRampRate()};
        break;
    case 8: // kCompensatedNominalVoltage
        actual_value = double{motor_->GetVoltageCompensationNominalVoltage()};
        break;
    case 9: // kAltEncoderInverted
        if (altMode)
        {
            actual_value = bool{encoder_->GetInverted()};
        }
        break;
    case 10: // kEncoderAverageDepth
        if (!altMode)
        {
            actual_value = uint{encoder_->GetAverageDepth()};
        }
        break;
    case 11: // kAltEncoderAverageDepth
        if (altMode)
        {
            actual_value = uint{encoder_->GetAverageDepth()};
        }
        break;
    case 12: // kEncoderSampleDelta
        if (!altMode)
        {
            actual_value = uint{encoder_->GetMeasurementPeriod()};
        }
        break;
    case 13: // kAltEncoderSampleDelta
        if (altMode)
        {
            actual_value = uint{encoder_->GetMeasurementPeriod()};
        }
        break;
    case 14: // kPositionConversionFactor
        if (!altMode)
        {
            actual_value = double{};
        }
        break;
    case 15: // kAltEncoderPositionFactor
        if (altMode)
        {
            actual_value = double{};
        }
        break;
    case 16: // kVelocityConversionFactor
        if (!altMode)
        {
            actual_value = double{};
        }
        break;
    case 17: // kAltEncoderVelocityFactor
        if (altMode)
        {
            actual_value = double{};
        }
        break;
    case 18: // kP_0
        actual_value = double{};
        break;
    case 19: // kI_0
        actual_value = double{};
        break;
    case 20: // kD_0
        actual_value = double{};
        break;
    case 21: // kF_0
        actual_value = double{};
        break;
    case 22: // kIZone_0
        actual_value = double{};
        break;
    case 23: // kIMaxAccum_0
        actual_value = double{};
        break;
    case 24: // kDFilter_0
        actual_value = double{};
        break;
    case 25: // kOutputMin_0
        actual_value = double{};
        break;
    case 26: // kOutputMax_0
        actual_value = double{};
        break;
    case 27: // kSmartMotionMaxVelocity_0
        actual_value = double{};
        break;
    case 28: // kSmartMotionMaxAccel_0
        actual_value = double{};
        break;
    case 29: // kSmartMotionMinVelOutput_0
        actual_value = double{};
        break;
    case 30: // kSmartMotionAllowedClosedLoopError_0
        actual_value = double{};
        break;
    case 31: // kSmartMotionAccelStrategy_0
        actual_value = double{};
        break;
    case 32: // kP_1
        actual_value = double{};
        break;
    case 33: // kI_1
        actual_value = double{};
        break;
    case 34: // kD_1
        actual_value = double{};
        break;
    case 35: // kF_1
        actual_value = double{};
        break;
    case 36: // kIZone_1
        actual_value = double{};
        break;
    case 37: // kIMaxAccum_1
        actual_value = double{};
        break;
    case 38: // kDFilter_1
        actual_value = double{};
        break;
    case 39: // kOutputMin_1
        actual_value = double{};
        break;
    case 40: // kOutputMax_1
        actual_value = double{};
        break;
    case 41: // kSmartMotionMaxVelocity_1
        actual_value = double{};
        break;
    case 42: // kSmartMotionMaxAccel_1
        actual_value = double{};
        break;
    case 43: // kSmartMotionMinVelOutput_1
        actual_value = double{};
        break;
    case 44: // kSmartMotionAllowedClosedLoopError_1
        actual_value = double{};
        break;
    case 45: // kSmartMotionAccelStrategy_1
        actual_value = double{};
        break;
    }
        } });

    return false;
}

void SparkMax::ApplyConfig(const std::string_view key, const ConfigValue &value) noexcept
{
}
