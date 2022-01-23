#pragma once

#include "subsystems/SmartMotor.h"

namespace SparkMaxFactory
{
    std::unique_ptr<SmartMotorBase> CreateSparkMax(const int canId, const int encoderCounts = 0) noexcept;
}
