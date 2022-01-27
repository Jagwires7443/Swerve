#pragma once

#include <string_view>

#include "subsystems/SmartMotor.h"

namespace SparkMaxFactory
{
    std::unique_ptr<SmartMotorBase> CreateSparkMax(const std::string_view name, const int canId, const bool inverted, const int encoderCounts = 0) noexcept;
}

// Configuration:
