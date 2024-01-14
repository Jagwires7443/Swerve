#pragma once

#include "infrastructure/ShuffleboardWidgets.h"

#include <frc/DigitalInput.h>
#include <frc/DutyCycle.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <units/angle.h>

#include <functional>
#include <memory>
#include <optional>
#include <utility>

class AngleSensor
{
public:
    AngleSensor(const int channel, const int alignment) noexcept;

    AngleSensor(const AngleSensor &) = delete;
    AngleSensor &operator=(const AngleSensor &) = delete;

    void Periodic() noexcept;

    void ShuffleboardCreate(frc::ShuffleboardContainer &container,
                            std::function<std::pair<units::angle::degree_t, units::angle::degree_t>()> getCommandedAndEncoderPositions = nullptr) noexcept;

    int GetAlignment() noexcept { return alignment_; }

    void SetAlignment(const int alignment) noexcept { alignment_ = alignment; }

    std::optional<units::angle::degree_t> GetAbsolutePosition() noexcept;

    std::optional<int> GetAbsolutePositionWithoutAlignment() noexcept;

private:
    // Range is [-2048, +2048).
    std::optional<int> GetAbsolutePosition(const int frequency, const double output, const bool applyOffset) noexcept;

    int alignment_{0};

    std::unique_ptr<frc::DigitalInput> digitalInput_;
    std::unique_ptr<frc::DutyCycle> dutyCycle_;

    std::function<std::pair<units::angle::degree_t, units::angle::degree_t>()> getCommandedAndEncoderPositionsF_{nullptr};
    HeadingGyro headingGyro_;

    bool shuffleboard_{false};
    frc::SimpleWidget *frequencyUI_{nullptr};
    frc::SimpleWidget *commandDiscrepancyUI_{nullptr};
    frc::SimpleWidget *statusUI_{nullptr};
    frc::ComplexWidget *headingUI_{nullptr};
    frc::SimpleWidget *commandedUI_{nullptr};
    frc::SimpleWidget *positionUI_{nullptr};
    frc::SimpleWidget *outputUI_{nullptr};
    frc::SimpleWidget *encoderDiscrepancyUI_{nullptr};
    frc::SimpleWidget *alignmentUI_{nullptr};
};
