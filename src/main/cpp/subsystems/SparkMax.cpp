// XXX use count, instead of bool for cumulative counts, include reset count
// XXX No throws, instead function simply increments error counter for later reporting

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
#include <tuple>

#include <rev/CANSparkMax.h>
#include <rev/CANSparkMaxLowLevel.h>
#include <rev/RelativeEncoder.h>
#include <rev/REVLibError.h>
#include <rev/SparkMaxPIDController.h>

#include <frc/RobotController.h>
#include <frc/shuffleboard/ShuffleboardWidget.h>

// Spark Max is a smart motor controller, which communicates via the CAN bus.
// There are periodic packets which flow both to and from this controller.  API
// calls either piggyback on this ongoing exchange of information (in which
// case they are fast and essentially free) or, they require a round-trip
// packet exchange (in which case, they are slow and it is important to not
// attempt too much of this type of exchange in the context of any single
// periodic interval).  In general, the latter type of API call will return
// a rev::REVLibError, while the former will not.
// See <https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces>
// for additional detail.  Note that faults are periodically reported, at a
// high frequency (by default).  This may be leveraged in order to detect
// problems, including situations in which the controller reboots.  This is
// not an especially friendly interface, though it is functional.  One of the
// main points of the SmartMotor and SparkMax classes is to encapsulate as much
// of this complexity as possible, including robust error handling, offering an
// API which is much easier to use correctly.

namespace
{
    class SparkMax : public SmartMotorBase
    {
    public:
        SparkMax(const std::string_view name, const int canId, const bool inverted, const int encoderCounts) noexcept : name_{name}, canId_{canId}, inverted_{inverted}, encoderCounts_{encoderCounts} {}

        SparkMax(const SparkMax &) = delete;
        SparkMax &operator=(const SparkMax &) = delete;

#if 0
        void ConfigIndex() noexcept
        {
            for (const auto &elem : SparkMaxFactory::configDefaults)
            {
                const auto kv = SparkMaxFactory::configDefaults.find(elem.first);

                const auto ndx = std::distance(kv, SparkMaxFactory::configDefaults.end());

                std::printf("Index: %s = %i\n", elem.first.c_str(), ndx);
            }
        }
#endif

        void SetConfig(const ConfigMap config) noexcept override
        {
            config_ = config;
            // XXX merge in {"kIdleMode", uint{1}}
        }

        void AddConfig(const ConfigMap config) noexcept override { config_.merge(ConfigMap(config)); }

        void CheckConfig() noexcept override
        {
            if (configLock_)
            {
                return;
            }

            configRead_ = true;
        }

        void ApplyConfig(bool burn) noexcept override
        {
            if (configLock_)
            {
                return;
            }

            if (burn)
            {
                configBurn_ = false;
            }
            else
            {
                configPush_ = false;
            }
        }

        void ClearFaults() noexcept override { faultBits_.reset(); }

        bool GetStatus() noexcept override
        {
            return motor_ && encoder_ && controller_ && faultBits_.none() && configGood_;
        }

        // Every method listed above does any work it has via Periodic().
        void Periodic() noexcept override;

        void ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                std::function<void(double)> control = nullptr,
                                std::function<void()> reset = nullptr) noexcept override;

        void SetIdleMode(const IdleMode mode) noexcept override;

        IdleMode GetIdleMode() noexcept override;

        void Stop() noexcept override;

        void Set(const double percent) noexcept override;

        double Get() noexcept override;

        void SetVoltage(const units::volt_t voltage) noexcept override;

        void SetCurrent(const units::ampere_t current) noexcept override;

        void SpecifyPosition(const double position) noexcept override;

        void SeekPosition(const double position) noexcept override;

        bool CheckPosition(const double tolerance) noexcept override;

        double GetPositionRaw() noexcept override;

        void SeekVelocity(const double velocity) noexcept override;

        bool CheckVelocity(const double tolerance) noexcept override;

        double GetVelocityRaw() noexcept override;

    private:
        // Parameters supplied though ctor.
        const std::string name_;
        const int canId_;
        const bool inverted_;
        const int encoderCounts_;

        // Underlying REV object holders.
        std::unique_ptr<rev::CANSparkMax> motor_;
        std::unique_ptr<rev::RelativeEncoder> encoder_;
        std::unique_ptr<rev::SparkMaxPIDController> controller_;

        // Shuffleboard UI elements, used by ShuffleboardCreate() and
        // ShuffleboardPeriodic() only.
        bool shuffleboard_ = false;
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

        // Shuffleboard UI control element mechanism.
        double previousControl_ = 0.0;
        std::function<void(double)> controlF_ = nullptr;
        std::function<void()> resetF_ = nullptr;

        // Configuration Parameters, set via SetConfig() and AddConfig().
        ConfigMap config_;
        ConfigMap updateConfig_;
        std::string configReporting_;
        std::string updateConfigReporting_;

        bool configReboot_ = true;
        bool configGood_ = false;
        bool configLock_ = false;
        bool configRead_ = false;
        bool configPush_ = false;
        bool configBurn_ = false;

        // Config parameters which are not individually settable.
        double outputRangeMin0_ = std::get<double>(SparkMaxFactory::configDefaults.at("kOutputMin_0"));
        double outputRangeMax0_ = std::get<double>(SparkMaxFactory::configDefaults.at("kOutputMax_0"));
        double outputRangeMin1_ = std::get<double>(SparkMaxFactory::configDefaults.at("kOutputMin_1"));
        double outputRangeMax1_ = std::get<double>(SparkMaxFactory::configDefaults.at("kOutputMax_1"));
        uint followerID_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kFollowerID"));
        uint followerConfig_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kFollowerConfig"));
        double currentChop_ = std::get<double>(SparkMaxFactory::configDefaults.at("kCurrentChop"));
        uint currentChopCycles_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kCurrentChopCycles"));
        uint smartCurrentStallLimit_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kSmartCurrentStallLimit"));
        uint smartCurrentFreeLimit_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kSmartCurrentFreeLimit"));
        uint smartCurrentConfig_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kSmartCurrentConfig"));
        uint status0_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kStatus0"));
        uint status1_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kStatus1"));
        uint status2_ = std::get<uint>(SparkMaxFactory::configDefaults.at("kStatus2"));

        std::bitset<16> faultBits_;
        uint64_t FPGATime_ = frc::RobotController::GetFPGATime();
        int iteration_ = 0;
        uint sequence_ = 0;

        double position_ = 0.0;
        double velocity_ = 0.0;

        void ShuffleboardPeriodic() noexcept;
        void ConfigPeriodic() noexcept;
        void DoSafely(const char *const what, std::function<void()> work, const bool check = true) noexcept;
        bool AnyError(const rev::REVLibError returnCode) noexcept;
        std::tuple<bool, bool, std::string> VerifyConfig(const std::string_view key, const ConfigValue &value) noexcept;
        std::string ApplyConfig(const std::string_view key, const ConfigValue &value) noexcept;
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

        firmwareInfo += firmwareVersionHex.str();
        firmwareInfo += "/";
        firmwareInfo += std::to_string(firmwareFieldMajor.to_ulong());
        firmwareInfo += ".";
        firmwareInfo += std::to_string(firmwareFieldMinor.to_ulong());
        firmwareInfo += ".";
        firmwareInfo += std::to_string(firmwareFieldBuild.to_ulong());
        firmwareInfo += "/\"";
        firmwareInfo += firmwareString;
        firmwareInfo += "\"";

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

    shuffleboard_ = true;
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

        // Note that "SpecifyPosition(0.0);" is called below.
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
    DoSafely("ShuffleboardPeriodic", [&]() -> void
             {
        temperature = motor_->GetMotorTemperature();
        faults = motor_->GetFaults();
        stickyFaults = motor_->GetStickyFaults();
        voltage = motor_->GetBusVoltage();
        current = motor_->GetOutputCurrent();
        percent = motor_->GetAppliedOutput(); // Actual setting
        speed = motor_->Get();                // Commanded setting

        distance = encoder_->GetPosition();        // Rotations
        velocity = encoder_->GetVelocity() / 60.0; // Rotations per second
        // The following semicolon is necessary to avoid mangling by the code
        // formatter.
        ; });

    if (reset)
    {
        // Logic above ensures that `position` is now zero; reset the
        // turning motor encoder to reflect this.
        SpecifyPosition(0.0);
    }

    temperatureUI_->GetEntry().SetDouble(temperature);
    statusUI_->GetEntry().SetBoolean(motor_ && encoder_ && controller_ && configGood_ && faults == 0);
    faultsUI_->GetEntry().SetString(FaultInfo(faults));
    stickyFaultsUI_->GetEntry().SetString(FaultInfo(stickyFaults));
    voltageUI_->GetEntry().SetDouble(voltage);
    currentUI_->GetEntry().SetDouble(current);
    percentUI_->GetEntry().SetDouble(percent);
    speedUI_->GetEntry().SetDouble(speed);
    distanceUI_->GetEntry().SetDouble(distance);
    velocityUI_->GetEntry().SetDouble(velocity);
}

// Config parameters are managed via a state machine, in order to spread slow
// calls out across multiple iterations of ConfigPeriodic().  CheckConfig()
// will return false until all requested operations have been completed; note
// that the only requested operations are ApplyConfig(), with the Boolean
// argument set each way.  This is because checking is done on an ongoing
// basis, so long as ConfigPeriodic() is being called.  CheckConfig() reports
// true only when there are no ongoing updates of config parameters and a
// check of all parameters has been completed subsequent to any update.
// ApplyConfig() simply resets the state machine and has no status to report.
// Both CheckConfig() and ApplyConfig() use `config_`, which holds the result
// of any/all prior calls to SetConfig() and/or AddConfig().  Note that the
// state maintained via Periodic() is also important in this context.

// XXX
// start (X = check, Y = apply (X + Y), Z = apply and burn (X + Y + Z))
// Y: block out normal operations which could affect config, block out Z
// Z: block out all normal operations, Y has no effect
// Z: 1) RestoreFactoryDefaults()
// Z: 2) redo ctor logic?
// X: 3) if encoderCount_ != 0 (alt encoder), verify GetCountsPerRevolution() matches
// 4) loop over parameters
//     X: check one param
//     Y: update one param, as needed
// X: update string and status value
// Y: update apply string
// Z: 5) BurnFlash()

// Periodic() normally runs at 50Hz, but this is a default and it is possible
// that this is running at another rate.  So, GetFPGATime() is used to work
// things out (more below), for management of any reboot and config parameters.
void SparkMax::Periodic() noexcept
{
    // Do any Shuffleboard work, if Shuffleboard has been set up.
    if (shuffleboard_)
    {
        ShuffleboardPeriodic();
    }

    // This call should be safe and "free", in that it only reports information
    // collected from periodic status frames.  To capture any transient events,
    // GetStickyFaults() is used instead of GetFaults().  There is no
    // expectation that fault information is instataeously accurate, it is only
    // used for reporting and to detect any motor controller reboots.  Make
    // things extra sticky using bitwise OR.  This is cleared by ClearFaults().
    DoSafely(
        "GetStickyFaults", [&]() -> void
        {
            if (motor_)
            {
                faultBits_ |= std::bitset<16>(motor_->GetStickyFaults());
            } },
        false);

    // In microseconds.  At 50Hz, this will nominally tick by increments of 20.
    // The object here is too only continue at a rate of around 3Hz, and to
    // spread things around so that the aggregate work of all motor controllers
    // is done evenly; to avoid synchonizing many motor controllers so they all
    // do this work during the same period.  The CAN ID (mod 16) is used to
    // facillitate this spread, along with a counter to track Periodic() calls.
    // If the rate for Periodic() is too low (< ~20Hz), things are going to run
    // slowly, but the robot is going to be generally sluggish.
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    uint deltaTime = FPGATime - FPGATime_;

    if (++iteration_ % 16 != canId_ % 16 || deltaTime < 250)
    {
        return;
    }

    FPGATime_ = FPGATime;
    iteration_ = canId_ % 16;

    // XXX Keep counters from faultBits (not underscore), these persist and are reported...

    ConfigPeriodic();
}

// Continue the post reset state machine if appropriate.  This will construct
// the triple set of REV objects associated with this SparkMax, clear any motor
// controller faults, and restore volatile state.  After things are at this
// point, persistent config parameters are handled by another state machine, as
// appropriate.
void SparkMax::ConfigPeriodic() noexcept
{
    // First stage of state machine; this has to be there before going further.
    if (!motor_)
    {
        DoSafely(
            "motor_", [&]() -> void
            {
            motor_ = std::make_unique<rev::CANSparkMax>(canId_, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
            if (!motor_)
            {
                throw std::runtime_error("motor_");
            }

            // Does not get sent to the motor controller, done locally.
            motor_->SetInverted(inverted_); },
            false);

        return;
    }

    // Second stage of state machine, as above.  Might or might not be slow, so
    // give it a stage to cover worst case.
    if (!encoder_)
    {
        DoSafely(
            "encoder_", [&]() -> void
            {
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
            } },
            false);

        return;
    }

    // Third stage of state machine, similar to prior stages.  Always fast?
    if (!controller_)
    {
        DoSafely(
            "controller_", [&]() -> void
            {
            controller_ = std::make_unique<rev::SparkMaxPIDController>(motor_->GetPIDController());
            if (!controller_)
            {
                throw std::runtime_error("controller_");
            } },
            false);

        return;
    }

    // This is the only reliable way to extract the reset status; again, safe
    // and "free".  Start the post reset state machine.
    DoSafely("GetStickyFault", [&]() -> void
             {
        if (motor_->GetStickyFault(rev::CANSparkMax::FaultID::kHasReset))
        {
            configReboot_ = true;
            configGood_ = false;
            sequence_ = 0;
        } });

    if (configReboot_)
    {
        // Optimized for setting periodic frame period, not appropriate for
        // general cofig parameters.
        auto periodicFramePeriod = [&](const std::string_view k, const uint v) -> bool
        {
            // Check against default value (other results are not
            // interesting in this specific context, absent coding errors).
            // This is fast/"free", so it does not need it's own state.
            const std::tuple<bool, bool, std::string> verify = VerifyConfig(k, uint{v});

            if (std::get<1>(verify))
            {
                const std::string apply = ApplyConfig(k, uint{v});

                if (!apply.empty())
                {
                    return false;
                }
            }

            return true;
        };

        switch (sequence_)
        {
        case 0:
            // First, clear the fault, so progress can be made.
            DoSafely("ClearFaults", [&]() -> void
                     {
                if (AnyError(motor_->ClearFaults()))
                    {
                        return;
                    } });

            // Next state depends on type of encoder.
            if (encoderCounts_ != 0)
            {
                sequence_ = 1;
            }
            else
            {
                sequence_ = 3;
            }

            return;
        case 1:
            // For alternate encoder, need to set it up as feedback source.
            DoSafely("SetFeedbackDevice", [&]() -> void
                     {
                if (AnyError(controller_->SetFeedbackDevice(*encoder_)))
                {
                    throw std::runtime_error("SetFeedbackDevice()");
                } });

            sequence_ = 2;

            return;
        case 2:
            // For alternate encoder, check counts per revolution is correct.
            DoSafely("GetCountsPerRevolution", [&]() -> void
                     {
                if (encoderCounts_ < 0 || static_cast<uint32_t>(encoderCounts_) != encoder_->GetCountsPerRevolution())
                {
                    throw std::runtime_error("GetCountsPerRevolution()");
                } });

            sequence_ = 3;

            return;
        case 3:
            // Restore volatile state: status frame 0 period.
            if (periodicFramePeriod("kStatus0", status0_))
            {
                sequence_ = 4;
            }

            return;
        case 4:
            // Restore volatile state: status frame 1 period.
            if (periodicFramePeriod("kStatus1", status1_))
            {
                sequence_ = 5;
            }

            return;
        case 5:
            // Restore volatile state: status frame 2 period.
            if (periodicFramePeriod("kStatus2", status2_))
            {
                configReboot_ = false;
                configGood_ = true;
                sequence_ = 0;
            }

            return;
        }
    }

    if (!configRead_ && !configPush_ && !configBurn_)
    {
        return;
    }

    // At this point, handle managed config parameters.
    if (!configLock_)
    {
        configGood_ = true;
        configLock_ = true;
        sequence_ = 0;
        updateConfig_.clear();
        updateConfigReporting_.clear();
    }

    if (sequence_ < config_.size())
    {
        auto it = config_.begin();
        std::advance(it, sequence_);

        // If false, first result indicates config is bad (read) or needs to be
        // applied (push).  If true, second result indicates config needs to be
        // applied (burn).  Third result is for reporting.
        const std::tuple<bool, bool, std::string> verify = VerifyConfig(it->first, it->second);

        if (!std::get<0>(verify))
        {
            configGood_ = false;

            if (!configBurn_)
            {
                updateConfig_[it->first] = it->second;
            }
        }

        if (configBurn_ && std::get<1>(verify))
        {
            updateConfig_[it->first] = it->second;
        }

        if (!std::get<2>(verify).empty())
        {
            if (!updateConfigReporting_.empty())
            {
                updateConfigReporting_ += " ";
            }

            updateConfigReporting_ += std::get<2>(verify);
        }

        sequence_++;

        return;
    }

    if (updateConfigReporting_ != configReporting_)
    {
        configReporting_ = updateConfigReporting_;

        if (!configReporting_.empty())
        {
            std::printf("SparkMax[%i] %s config: %s.\n", canId_, name_.c_str(), configReporting_.c_str());
        }
    }

    configLock_ = false;
    sequence_ = 0;
}

void SparkMax::SetIdleMode(const SmartMotorBase::IdleMode mode) noexcept
{
    if (configLock_)
    {
        return;
    }

    const rev::CANSparkMax::IdleMode tmp = (mode == SmartMotorBase::IdleMode::kBrake) ? rev::CANSparkMax::IdleMode::kBrake : rev::CANSparkMax::IdleMode::kCoast;

    DoSafely("SetIdleMode", [&]() -> void
             { motor_->SetIdleMode(tmp); });
}

SmartMotorBase::IdleMode SparkMax::GetIdleMode() noexcept
{
    rev::CANSparkMax::IdleMode mode = motor_->GetIdleMode();

    DoSafely("GetIdleMode", [&]() -> void
             { mode = motor_->GetIdleMode(); });

    return mode == rev::CANSparkMax::IdleMode::kBrake ? SmartMotorBase::IdleMode::kBrake : SmartMotorBase::IdleMode::kCoast;
}

void SparkMax::Stop() noexcept
{
    position_ = 0.0;
    velocity_ = 0.0;

    DoSafely("Stop", [&]() -> void
             { motor_->StopMotor(); });
}

void SparkMax::Set(const double percent) noexcept
{
    position_ = 0.0;
    velocity_ = 0.0;

    DoSafely("Set", [&]() -> void
             { motor_->Set(percent); });
}

double SparkMax::Get() noexcept
{
    double result = 0.0;

    DoSafely("Get", [&]() -> void
             { result = motor_->Get(); });

    return result;
}

// This is based on voltage compensation, rather than feedback.
void SparkMax::SetVoltage(const units::volt_t voltage) noexcept
{
    position_ = 0.0;
    velocity_ = 0.0;

    DoSafely("SetVoltage", [&]() -> void
             { motor_->SetVoltage(voltage); });
}

// Note that current control uses PID parameter set 0, which is normally used
// for position.  It is unlikely both current and position control would be
// used with the same mechanism, so this is OK.  Any error is tracked, but is
// not propagated from this context (similar to Set).
void SparkMax::SetCurrent(const units::ampere_t current) noexcept
{
    position_ = 0.0;
    velocity_ = 0.0;

    DoSafely("SetCurrent", [&]() -> void
             { (void)AnyError(controller_->SetReference(current.to<double>(), rev::CANSparkMax::ControlType::kCurrent, 0)); });
}

// Note that any error is tracked, but is not propagated from this context.
void SparkMax::SpecifyPosition(const double position) noexcept
{
    DoSafely("SpecifyPosition", [&]() -> void
             { (void)AnyError(encoder_->SetPosition(position)); });
}

void SparkMax::SeekPosition(const double position) noexcept
{
    position_ = position;
    velocity_ = 0.0;

    DoSafely("SeekPosition", [&]() -> void
             { (void)AnyError(controller_->SetReference(position, rev::CANSparkMax::ControlType::kSmartMotion, 0)); });
}

bool SparkMax::CheckPosition(const double tolerance) noexcept
{
    double error = GetPositionRaw() - position_;

    return error >= -tolerance && error < tolerance;
}

double SparkMax::GetPositionRaw() noexcept
{
    double result = 0.0;

    DoSafely("GetPosition", [&]() -> void
             { result = encoder_->GetPosition(); });

    return result;
}

void SparkMax::SeekVelocity(const double velocity) noexcept
{
    position_ = 0.0;
    velocity_ = velocity;

    DoSafely("SeekVelocity", [&]() -> void
             { (void)AnyError(controller_->SetReference(velocity, rev::CANSparkMax::ControlType::kSmartVelocity, 1)); });
}

bool SparkMax::CheckVelocity(const double tolerance) noexcept
{
    double error = GetVelocityRaw() - velocity_;

    return error >= -tolerance && error < tolerance;
}

double SparkMax::GetVelocityRaw() noexcept
{
    double result = 0.0;

    DoSafely("GetVelocity", [&]() -> void
             { result = encoder_->GetVelocity(); });

    return result;
}

bool SparkMax::AnyError(const rev::REVLibError returnCode) noexcept
{
    // XXX track errors
    switch (returnCode)
    {
    case rev::REVLibError::kOk:
        return false;
    case rev::REVLibError::kError:
        break;
    case rev::REVLibError::kTimeout:
        break;
    case rev::REVLibError::kNotImplemented:
        break;
    case rev::REVLibError::kHALError:
        break;
    case rev::REVLibError::kCantFindFirmware:
        break;
    case rev::REVLibError::kFirmwareTooOld:
        break;
    case rev::REVLibError::kFirmwareTooNew:
        break;
    case rev::REVLibError::kParamInvalidID:
        break;
    case rev::REVLibError::kParamMismatchType:
        break;
    case rev::REVLibError::kParamAccessMode:
        break;
    case rev::REVLibError::kParamInvalid:
        break;
    case rev::REVLibError::kParamNotImplementedDeprecated:
        break;
    case rev::REVLibError::kFollowConfigMismatch:
        break;
    case rev::REVLibError::kInvalid:
        break;
    case rev::REVLibError::kSetpointOutOfRange:
        break;
    case rev::REVLibError::kUnknown:
        break;
    case rev::REVLibError::kCANDisconnected:
        break;
    case rev::REVLibError::kDuplicateCANId:
        break;
    case rev::REVLibError::kInvalidCANId:
        break;
    case rev::REVLibError::kSparkMaxDataPortAlreadyConfiguredDifferently:
        break;
    }

    return true;
}

void SparkMax::DoSafely(const char *const what, std::function<void()> work, const bool check) noexcept
{
    try
    {
        if (check && (!motor_ || !encoder_ || !controller_))
        {
            motor_ = nullptr;
            encoder_ = nullptr;
            controller_ = nullptr;
            sequence_ = 0;
        }
        else
        {
            work();
        }
    }
    catch (const std::exception &e)
    {
        motor_ = nullptr;
        encoder_ = nullptr;
        controller_ = nullptr;
        sequence_ = 0;

        std::printf("SparkMax[%i] %s %s exception: %s.\n", canId_, name_.c_str(), what, e.what());
    }
    catch (...)
    {
        motor_ = nullptr;
        encoder_ = nullptr;
        controller_ = nullptr;
        sequence_ = 0;

        std::printf("SparkMax[%i] %s %s unknown exception.\n", canId_, name_.c_str(), what);
    }
}

// Check a single configuration parameter against a specific value; returns
// three pieces of information: 1) bool, value is positively verified; 2) bool,
// value needs to be set following a call to RestoreFactoryDefaults(); 3)
// string, text to report that may prompt going through procedure to burn
// config parameters.  This information is used to build a list of parameters
// to be used in the context of ApplyConfig(), to concatenate a string to
// report the results of CheckConfig(), and to roll up to the result for this
// latter function.
std::tuple<bool, bool, std::string> SparkMax::VerifyConfig(const std::string_view key, const ConfigValue &value) noexcept
{
    const auto kv = SparkMaxFactory::configDefaults.find(std::string(key));

    if (kv == SparkMaxFactory::configDefaults.end())
    {
        return std::make_tuple(false, false, "Invalid");
    }

    ConfigValue default_value = kv->second;

    // Have requested and default values.  Now, attempt to get current value.
    std::optional<ConfigValue> actual_value;

    // In order to avoid switching on a string, obtain the index into defaults.
    // The complication here is that maps are sorted, so the indices are not in
    // an easy-to-maintain order.
    const auto ndx = std::distance(kv, SparkMaxFactory::configDefaults.end());

    // Some settings only apply to one encoder type or the other.
    const bool altMode = (encoderCounts_ != 0);

    // Only true for a non-default follower (which cannot be verified).
    bool follower = false;

    // This will be displayed, in the event there is an issue with the setting.
    std::string name = "Unknown";

    // In general, these are slow (round-trip to the controller) API calls.
    DoSafely("Get config parameter", [&]() -> void
             {
    switch (ndx)
    {
    // These first three are not real config parameters; just act as though
    // they verified correctly.  Periodic() is responsible for setting these,
    // and there is no way to verify them.
    case 4:
        name = "Status0";
        actual_value = uint{status0_};
        break;
    case 3:
        name = "Status1";
        actual_value = uint{status1_};
        break;
    case 2:
        name = "Status2";
        actual_value = uint{status2_};
        break;
    case 58:
        name = "Firmware Version (";
        {
            const uint val = uint{motor_->GetFirmwareVersion()};
            const std::string str = motor_->GetFirmwareString();

            name += FirmwareInfo(val, str);
            actual_value = val;
        }
        name += ")";
        break;
    case 30:
        name = "IdleMode";
        {
            const rev::CANSparkMax::IdleMode tmp{motor_->GetIdleMode()};

            if (tmp == rev::CANSparkMax::IdleMode::kCoast)
            {
                actual_value = uint{0};
            } else if (tmp == rev::CANSparkMax::IdleMode::kBrake)
            {
                actual_value = uint{1};

                // Force update for non-standard default.
                default_value = uint{0};
            }
        }
        break;
    // kLimitSwitchFwdPolarity = 29
    // kLimitSwitchRevPolarity = 28
    // kHardLimitFwdEn = 38
    // kHardLimitRevEn = 37
    case 39:
        name = "FollowerID";
        if (motor_->IsFollower())
        {
            follower = true;
        }
        else
        {
            actual_value = uint{0};
        }
        break;
    case 40:
        name = "FollowerConfig";
        if (motor_->IsFollower())
        {
            follower = true;
        }
        else
        {
            actual_value = uint{0};
        }
        break;
    case 6:
        name = "SoftLimitFwd";
        actual_value = double{motor_->GetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward)};
        break;
    case 5:
        name = "SoftLimitRev";
        actual_value = double{motor_->GetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse)};
        break;
    case 20:
        name = "RampRate";
        actual_value = double{motor_->GetOpenLoopRampRate()};
        break;
    case 52:
        name = "ClosedLoopRampRate";
        actual_value = double{motor_->GetClosedLoopRampRate()};
        break;
    case 51:
        name = "CompensatedNominalVoltage";
        actual_value = double{motor_->GetVoltageCompensationNominalVoltage()};
        break;
    case 56:
        name = "AltEncoderInverted";
        if (altMode)
        {
            actual_value = bool{encoder_->GetInverted()};
        }
        break;
    case 44:
        name = "EncoderAverageDepth";
        if (!altMode)
        {
            actual_value = uint{encoder_->GetAverageDepth()};
        }
        break;
    case 57:
        name = "AltEncoderAverageDepth";
        if (altMode)
        {
            actual_value = uint{encoder_->GetAverageDepth()};
        }
        break;
    case 43:
        name = "EncoderSampleDelta";
        if (!altMode)
        {
            actual_value = uint{encoder_->GetMeasurementPeriod()};
        }
        break;
    case 54:
        name = "AltEncoderSampleDelta";
        if (altMode)
        {
            actual_value = uint{encoder_->GetMeasurementPeriod()};
        }
        break;
    case 21:
        name = "PositionConversionFactor";
        if (!altMode)
        {
            actual_value = double{encoder_->GetPositionConversionFactor()};
        }
        break;
    case 55:
        name = "AltEncoderPositionFactor";
        if (altMode)
        {
            actual_value = double{encoder_->GetPositionConversionFactor()};
        }
        break;
    case 1:
        name = "VelocityConversionFactor";
        if (!altMode)
        {
            actual_value = double{encoder_->GetVelocityConversionFactor()};
        }
        break;
    case 53:
        name = "AltEncoderVelocityFactor";
        if (altMode)
        {
            actual_value = double{encoder_->GetVelocityConversionFactor()};
        }
        break;
    case 50:
        name = "kCurrentChop";
        follower = true;
        break;
    case 49:
        name = "kCurrentChopCycles";
        follower = true;
        break;
    case 17:
        name = "kSmartCurrentStallLimit";
        follower = true;
        break;
    case 18:
        name = "kSmartCurrentFreeLimit";
        follower = true;
        break;
    case 19:
        name = "kSmartCurrentConfig";
        follower = true;
        break;
    case 23:
        name = "P_0";
        actual_value = double{controller_->GetP(0)};
        break;
    case 32:
        name = "I_0";
        actual_value = double{controller_->GetI(0)};
        break;
    case 46:
        name = "D_0";
        actual_value = double{controller_->GetD(0)};
        break;
    case 42:
        name = "F_0";
        actual_value = double{controller_->GetFF(0)};
        break;
    case 34:
        name = "IZone_0";
        actual_value = double{controller_->GetIZone(0)};
        break;
    case 36:
        name = "IMaxAccum_0";
        actual_value = double{controller_->GetIMaxAccum(0)};
        break;
    case 48:
        name = "DFilter_0";
        actual_value = double{controller_->GetDFilter(0)};
        break;
    case 25:
        name = "OutputMin_0";
        outputRangeMin0_ = double{controller_->GetOutputMin(0)};
        actual_value = outputRangeMin0_;
        break;
    case 27:
        name = "OutputMax_0";
        outputRangeMax0_ = double{controller_->GetOutputMax(0)};
        actual_value = outputRangeMax0_;
        break;
    case 10:
        name = "SmartMotionMaxVelocity_0";
        actual_value = double{controller_->GetSmartMotionMaxVelocity(0)};
        break;
    case 12:
        name = "SmartMotionMaxAccel_0";
        actual_value = double{controller_->GetSmartMotionMaxAccel(0)};
        break;
    case 8:
        name = "SmartMotionMinVelOutput_0";
        actual_value = double{controller_->GetSmartMotionMinOutputVelocity(0)};
        break;
    case 14:
        name = "SmartMotionAllowedClosedLoopError_0";
        actual_value = double{controller_->GetSmartMotionAllowedClosedLoopError(0)};
        break;
    case 16:
        name = "SmartMotionAccelStrategy_0";
        {
            const rev::SparkMaxPIDController::AccelStrategy tmp{controller_->GetSmartMotionAccelStrategy(0)};

            if (tmp == rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal)
            {
                actual_value = uint{0};
            }
            else if (tmp == rev::SparkMaxPIDController::AccelStrategy::kSCurve)
            {
                actual_value = uint{1};
            }
        }
        break;
    case 22:
        name = "P_1";
        actual_value = double{controller_->GetP(1)};
        break;
    case 31:
        name = "I_1";
        actual_value = double{controller_->GetI(1)};
        break;
    case 45:
        name = "D_1";
        actual_value = double{controller_->GetD(1)};
        break;
    case 41:
        name = "F_1";
        actual_value = double{controller_->GetFF(1)};
        break;
    case 33:
        name = "IZone_1";
        actual_value = double{controller_->GetIZone(1)};
        break;
    case 35:
        name = "IMaxAccum_1";
        actual_value = double{controller_->GetIMaxAccum(1)};
        break;
    case 47:
        name = "DFilter_1";
        actual_value = double{controller_->GetDFilter(1)};
        break;
    case 24:
        name = "OutputMin_1";
        outputRangeMin1_ = double{controller_->GetOutputMin(1)};
        actual_value = outputRangeMin0_;
        break;
    case 26:
        name = "OutputMax_1";
        outputRangeMax1_ = double{controller_->GetOutputMax(1)};
        actual_value = outputRangeMax0_;
        break;
    case 9:
        name = "SmartMotionMaxVelocity_1";
        actual_value = double{controller_->GetSmartMotionMaxVelocity(1)};
        break;
    case 11:
        name = "SmartMotionMaxAccel_1";
        actual_value = double{controller_->GetSmartMotionMaxAccel(1)};
        break;
    case 7:
        name = "SmartMotionMinVelOutput_1";
        actual_value = double{controller_->GetSmartMotionMinOutputVelocity(1)};
        break;
    case 13:
        name = "SmartMotionAllowedClosedLoopError_1";
        actual_value = double{controller_->GetSmartMotionAllowedClosedLoopError(1)};
        break;
    case 15:
        name = "SmartMotionAccelStrategy_1";
        {
            const rev::SparkMaxPIDController::AccelStrategy tmp{controller_->GetSmartMotionAccelStrategy(1)};

            if (tmp == rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal)
            {
                actual_value = uint{0};
            }
            else if (tmp == rev::SparkMaxPIDController::AccelStrategy::kSCurve)
            {
                actual_value = uint{1};
            }
        }
        break;
    } });

    // Now, have the desired `value`, the `default_value`, and may have the
    // `actual_value`.  Additionally, `name` contains text to display when it
    // is appropriate to report a mismatch (or if a match cannot be verified,
    // which should only occur for non-default follower ID or config -- in this
    // case, `follower` will be true).

    // Special case for follower config (only partially verified, so prompt
    // string is not empty).
    if (follower)
    {
        return std::make_tuple(true, true, name);
    }

    // Something unexpected has occurred, so maximize the likelihood of update.
    if (!actual_value)
    {
        return std::make_tuple(false, true, name);
    }

    if (*actual_value == value)
    {
        name.clear();
    }

    return std::make_tuple(*actual_value == value, value != default_value, name);
}

std::string SparkMax::ApplyConfig(const std::string_view key, const ConfigValue &value) noexcept
{
    const auto kv = SparkMaxFactory::configDefaults.find(std::string(key));

    if (kv == SparkMaxFactory::configDefaults.end())
    {
        return std::string("Invalid");
    }

    // In order to avoid switching on a string, obtain the index into defaults.
    // The complication here is that maps are sorted, so the indices are not in
    // an easy-to-maintain order.
    const auto ndx = std::distance(kv, SparkMaxFactory::configDefaults.end());

    // Some settings only apply to one encoder type or the other.
    const bool altMode = (encoderCounts_ != 0);

    // This will be displayed, in the event there is an issue with the setting.
    std::string name = "Unknown";

    const bool *const pbool = std::get_if<bool>(&value);
    const uint *const puint = std::get_if<uint>(&value);
    const double *const pdouble = std::get_if<double>(&value);

    // In general, these are slow (round-trip to the controller) API calls.
    DoSafely("Set config parameter", [&]() -> void
             {
    switch (ndx)
    {
    // These first three are not real config parameters; store them so they may
    // be applied after any motor controller reboot.
    case 4:
        name = "Status0";
        if (puint)
        {
            status0_ = *puint;
            if (!AnyError(motor_->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, status0_)))
            {
                name.clear();
            }
        }
        break;
    case 3:
        name = "Status1";
        if (puint)
        {
            status1_ = *puint;
            if (!AnyError(motor_->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, status1_)))
            {
                name.clear();
            }
        }
        break;
    case 2:
        name = "Status2";
        if (puint)
        {
            status2_ = *puint;
            if (!AnyError(motor_->SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, status2_)))
            {
                name.clear();
            }
        }
        break;
    case 58:
        name = "Firmware Version";
        // This cannot be set here.
        break;
    case 30:
        name = "IdleMode";
        if (puint && *puint <= 1)
        {
            const rev::CANSparkMax::IdleMode tmp = (*puint == 0 ? rev::CANSparkMax::IdleMode::kCoast : rev::CANSparkMax::IdleMode::kBrake);

            if (!AnyError(motor_->SetIdleMode(tmp)))
            {
                name.clear();
            }
        }
        break;
    // kLimitSwitchFwdPolarity = 29
    // kLimitSwitchRevPolarity = 28
    // kHardLimitFwdEn = 38
    // kHardLimitRevEn = 37
    case 39:
        name = "FollowerID";
        if (puint)
        {
            followerID_ = *puint;

            const rev::CANSparkMax::ExternalFollower tmp{(int)followerID_, (int)followerConfig_};
            const int deviceID = followerID_ & 0x3f;
            const bool invert = (followerConfig_ & 0x00040000) != 0;

            if (followerConfig_ != 0)
            {
                if (!AnyError(motor_->Follow(tmp, deviceID, invert)))
                {
                    name.clear();
                }
            }
        }
        break;
    case 40:
        name = "FollowerConfig";
        if (puint)
        {
            followerConfig_ = *puint;

            const rev::CANSparkMax::ExternalFollower tmp{(int)followerID_, (int)followerConfig_};
            const int deviceID = followerID_ & 0x3f;
            const bool invert = (followerConfig_ & 0x00040000) != 0;

            if (followerID_ != 0)
            {
                if (!AnyError(motor_->Follow(tmp, deviceID, invert)))
                {
                    name.clear();
                }
            }
        }
        break;
    case 6:
        name = "SoftLimitFwd";
        if (pdouble)
        {
            if (!AnyError(motor_->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, *pdouble)))
            {
                name.clear();
            }
        }
        break;
    case 5:
        name = "SoftLimitRev";
        if (pdouble)
        {
            if (!AnyError(motor_->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, *pdouble)))
            {
                name.clear();
            }
        }
        break;
    case 20:
        name = "RampRate";
        if (pdouble)
        {
            if (!AnyError(motor_->SetOpenLoopRampRate(*pdouble)))
            {
                name.clear();
            }
        }
        break;
    case 52:
        name = "ClosedLoopRampRate";
        if (pdouble)
        {
            if (!AnyError(motor_->SetClosedLoopRampRate(*pdouble)))
            {
                name.clear();
            }
        }
        break;
    case 51:
        name = "CompensatedNominalVoltage";
        if (pdouble)
        {
            if (*pdouble == 0.0)
            {
                if (!AnyError(motor_->DisableVoltageCompensation()))
                {
                    name.clear();
                }
            }
            else
            {
                if (!AnyError(motor_->EnableVoltageCompensation(*pdouble)))
                {
                    name.clear();
                }
            }
        }
        break;
    case 56:
        name = "AltEncoderInverted";
        if (pbool && altMode)
        {
            if (!AnyError(encoder_->SetInverted(*pbool)))
            {
                name.clear();
            }
        }
        break;
    case 44:
        name = "EncoderAverageDepth";
        if (puint && !altMode)
        {
            if (!AnyError(encoder_->SetAverageDepth(*puint)))
            {
                name.clear();
            }
        }
        break;
    case 57:
        name = "AltEncoderAverageDepth";
        if (puint && altMode)
        {
            if (!AnyError(encoder_->SetAverageDepth(*puint)))
            {
                name.clear();
            }
        }
        break;
    case 43:
        name = "EncoderSampleDelta";
        if (puint && !altMode)
        {
            if (!AnyError(encoder_->SetMeasurementPeriod(*puint)))
            {
                name.clear();
            }
        }
        break;
    case 54:
        name = "AltEncoderSampleDelta";
        if (puint && altMode)
        {
            if (!AnyError(encoder_->SetMeasurementPeriod(*puint)))
            {
                name.clear();
            }
        }
        break;
    case 21:
        name = "PositionConversionFactor";
        if (pdouble && !altMode)
        {
            if (!AnyError(encoder_->SetPositionConversionFactor(*pdouble)))
            {
                name.clear();
            }
        }
        break;
    case 55:
        name = "AltEncoderPositionFactor";
        if (pdouble && altMode)
        {
            if (!AnyError(encoder_->SetPositionConversionFactor(*pdouble)))
            {
                name.clear();
            }
        }
        break;
    case 1:
        name = "VelocityConversionFactor";
        if (pdouble && !altMode)
        {
            if (!AnyError(encoder_->SetVelocityConversionFactor(*pdouble)))
            {
                name.clear();
            }
        }
        break;
    case 53:
        name = "AltEncoderVelocityFactor";
        if (pdouble && altMode)
        {
            if (!AnyError(encoder_->SetVelocityConversionFactor(*pdouble)))
            {
                name.clear();
            }
        }
        break;
    case 50:
        name = "kCurrentChop";
        if (pdouble)
        {
            currentChop_ = *pdouble;
            if (!AnyError(motor_->SetSecondaryCurrentLimit(currentChop_, currentChopCycles_)))
            {
                name.clear();
            }
        }
        break;
    case 49:
        name = "kCurrentChopCycles";
        if (puint)
        {
            currentChopCycles_ = *puint;
            if (!AnyError(motor_->SetSecondaryCurrentLimit(currentChop_, currentChopCycles_)))
            {
                name.clear();
            }
        }
        break;
    case 17:
        name = "kSmartCurrentStallLimit";
        if (puint)
        {
            smartCurrentStallLimit_ = *puint;
            if (!AnyError(motor_->SetSmartCurrentLimit(smartCurrentStallLimit_, smartCurrentFreeLimit_, smartCurrentConfig_)))
            {
                name.clear();
            }
        }
        break;
    case 18:
        name = "kSmartCurrentFreeLimit";
        if (puint)
        {
            smartCurrentFreeLimit_ = *puint;
            if (!AnyError(motor_->SetSmartCurrentLimit(smartCurrentStallLimit_, smartCurrentFreeLimit_, smartCurrentConfig_)))
            {
                name.clear();
            }
        }
        break;
    case 19:
        name = "kSmartCurrentConfig";
        if (puint)
        {
            smartCurrentConfig_ = *puint;
            if (!AnyError(motor_->SetSmartCurrentLimit(smartCurrentStallLimit_, smartCurrentFreeLimit_, smartCurrentConfig_)))
            {
                name.clear();
            }
        }
        break;
    case 23:
        name = "P_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetP(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 32:
        name = "I_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetI(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 46:
        name = "D_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetD(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 42:
        name = "F_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetFF(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 34:
        name = "IZone_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetIZone(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 36:
        name = "IMaxAccum_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetIMaxAccum(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 48:
        name = "DFilter_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetDFilter(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 25:
        name = "OutputMin_0";
        if (pdouble)
        {
            outputRangeMin0_ = *pdouble;
            if (!AnyError(controller_->SetOutputRange(outputRangeMin0_, outputRangeMax0_, 0)))
            {
                name.clear();
            }
        }
        break;
    case 27:
        name = "OutputMax_0";
        if (pdouble)
        {
            outputRangeMax0_ = *pdouble;
            if (!AnyError(controller_->SetOutputRange(outputRangeMin0_, outputRangeMax0_, 0)))
            {
                name.clear();
            }
        }
        break;
    case 10:
        name = "SmartMotionMaxVelocity_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetSmartMotionMaxVelocity(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 12:
        name = "SmartMotionMaxAccel_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetSmartMotionMaxAccel(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 8:
        name = "SmartMotionMinVelOutput_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetSmartMotionMinOutputVelocity(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 14:
        name = "SmartMotionAllowedClosedLoopError_0";
        if (pdouble)
        {
            if (!AnyError(controller_->SetSmartMotionAllowedClosedLoopError(*pdouble, 0)))
            {
                name.clear();
            }
        }
        break;
    case 16:
        name = "SmartMotionAccelStrategy_0";
        if (puint && *puint <= 1)
        {
            const rev::SparkMaxPIDController::AccelStrategy tmp = (*puint == 0 ? rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal : rev::SparkMaxPIDController::AccelStrategy::kSCurve);

            if (!AnyError(controller_->SetSmartMotionAccelStrategy(tmp, 0)))
            {
                name.clear();
            }
        }
        break;
    case 22:
        name = "P_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetP(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 31:
        name = "I_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetI(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 45:
        name = "D_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetD(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 41:
        name = "F_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetFF(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 33:
        name = "IZone_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetIZone(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 35:
        name = "IMaxAccum_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetIMaxAccum(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 47:
        name = "DFilter_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetDFilter(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 24:
        name = "OutputMin_1";
        if (pdouble)
        {
            outputRangeMin0_ = *pdouble;
            if (!AnyError(controller_->SetOutputRange(outputRangeMin0_, outputRangeMax0_, 1)))
            {
                name.clear();
            }
        }
        break;
    case 26:
        name = "OutputMax_1";
        if (pdouble)
        {
            outputRangeMax0_ = *pdouble;
            if (!AnyError(controller_->SetOutputRange(outputRangeMin0_, outputRangeMax0_, 1)))
            {
                name.clear();
            }
        }
        break;
    case 9:
        name = "SmartMotionMaxVelocity_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetSmartMotionMaxVelocity(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 11:
        name = "SmartMotionMaxAccel_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetSmartMotionMaxAccel(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 7:
        name = "SmartMotionMinVelOutput_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetSmartMotionMinOutputVelocity(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 13:
        name = "SmartMotionAllowedClosedLoopError_1";
        if (pdouble)
        {
            if (!AnyError(controller_->SetSmartMotionAllowedClosedLoopError(*pdouble, 1)))
            {
                name.clear();
            }
        }
        break;
    case 15:
        name = "SmartMotionAccelStrategy_1";
        if (puint && *puint <= 1)
        {
            const rev::SparkMaxPIDController::AccelStrategy tmp = (*puint == 0 ? rev::SparkMaxPIDController::AccelStrategy::kTrapezoidal : rev::SparkMaxPIDController::AccelStrategy::kSCurve);

            if (!AnyError(controller_->SetSmartMotionAccelStrategy(tmp, 1)))
            {
                name.clear();
            }
        }
        break;
    } });

    return name;
}
