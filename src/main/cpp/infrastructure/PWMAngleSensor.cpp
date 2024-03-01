#include "infrastructure/PWMAngleSensor.h"

#include "infrastructure/ShuffleboardWidgets.h"
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/MathUtil.h>

AngleSensor::AngleSensor(int deviceID, units::turn_t alignment) noexcept
    : canCoder_(deviceID) {
    // Configure the sensor range and other settings
    canCoder_.GetConfigurator().Apply(
        ctre::phoenix6::configs::MagnetSensorConfigs()
            .WithAbsoluteSensorRange(ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf)
            .WithMagnetOffset(alignment.value())
            // .WithSensorDirection(ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive)
    );
    int deviceId = canCoder_.GetDeviceID(); // Obtain the CANCoder device ID
    std::string deviceIdStr = std::to_string(deviceId); // Convert the device ID to string
}

void AngleSensor::Periodic() noexcept
{
    if (!shuffleboard_)
    {
        return;
    }

    units::angle::degree_t commandedPosition{0.0_deg};
    units::angle::degree_t encoderPosition{0.0_deg};

    // If possible, obtain commanded and encoder positions.
    if (getCommandedAndEncoderPositionsF_)
    {
        const auto pair = getCommandedAndEncoderPositionsF_();

        commandedPosition = std::get<0>(pair);
        encoderPosition = std::get<1>(pair);
    }

    const auto position = GetAbsolutePositionWithoutAlignment();
    const bool status = position.has_value();
    const units::turn_t alignment = GetAlignment();
    units::degree_t reportPosition{0};

    if (status)
    {
        reportPosition = position.value() + alignment;
    }

    const double heading = reportPosition.value();

    headingGyro_.Set(heading);

    double commandedError = heading - commandedPosition.to<double>();

    if (commandedError < -180.0)
    {
        commandedError += 360.0;
    }
    else if (commandedError >= 180.0)
    {
        commandedError -= 360.0;
    }

    double encoderError = heading - encoderPosition.to<double>();

    if (encoderError < -180.0)
    {
        encoderError += 360.0;
    }
    else if (encoderError >= 180.0)
    {
        encoderError -= 360.0;
    }

    commandDiscrepancyUI_->GetEntry()->SetDouble(commandedError);
    statusUI_->GetEntry()->SetBoolean(status);
    commandedUI_->GetEntry()->SetDouble(commandedPosition.to<double>());
    positionUI_->GetEntry()->SetDouble(reportPosition.to<double>());
    encoderDiscrepancyUI_->GetEntry()->SetDouble(encoderError);
    alignmentUI_->GetEntry()->SetDouble(alignment.to<double>());
}

void AngleSensor::ShuffleboardCreate(frc::ShuffleboardContainer &container,
                                     std::function<std::pair<units::angle::degree_t, units::angle::degree_t>()> getCommandedAndEncoderPositionsF) noexcept
{

    commandDiscrepancyUI_ = &container.Add("PID Error", 0)
                                 .WithPosition(0, 0);

    statusUI_ = &container.Add("Status", false)
                     .WithPosition(0, 1)
                     .WithWidget(frc::BuiltInWidgets::kBooleanBox);

    headingUI_ = &container.Add("Heading", headingGyro_)
                      .WithPosition(1, 0)
                      .WithWidget(frc::BuiltInWidgets::kGyro)
                      .WithProperties(wpi::StringMap<nt::Value>{
                          std::make_pair("Counter clockwise", nt::Value::MakeBoolean(true))});

    commandedUI_ = &container.Add("Commanded", 0)
                        .WithPosition(1, 1);

    positionUI_ = &container.Add("Position", 0)
                       .WithPosition(1, 2);

    encoderDiscrepancyUI_ = &container.Add("Motor Discrepancy", 0)
                                 .WithPosition(2, 0);

    alignmentUI_ = &container.Add("Alignment", 0)
                        .WithPosition(2, 1);

    getCommandedAndEncoderPositionsF_ = getCommandedAndEncoderPositionsF;

    shuffleboard_ = true;
}

units::turn_t AngleSensor::GetAlignment() noexcept {
    ctre::phoenix6::configs::MagnetSensorConfigs sensorConfig = ctre::phoenix6::configs::MagnetSensorConfigs();
    canCoder_.GetConfigurator().Refresh(sensorConfig);
    return (units::turn_t)sensorConfig.MagnetOffset;
};

void AngleSensor::SetAlignment(const units::turn_t alignment) noexcept {
    canCoder_.GetConfigurator().Apply(
        ctre::phoenix6::configs::MagnetSensorConfigs()
            .WithMagnetOffset(alignment.value())
    );
};

std::optional<units::angle::degree_t> AngleSensor::GetAbsolutePosition() noexcept {
    units::degree_t position = canCoder_.GetPosition().GetValue();  // Assuming this returns units::angle::degree_t
    int deviceId = canCoder_.GetDeviceID(); // Obtain the CANCoder device ID
    std::string deviceIdStr = std::to_string(deviceId); // Convert the device ID to string

    // Convert to radians and back to use MathUtil AngleModulus
    units::degree_t normalizedPosition = frc::AngleModulus(position);

    return normalizedPosition;
}

std::optional<units::angle::turn_t> AngleSensor::GetAbsolutePositionWithoutAlignment() noexcept
{
    units::degree_t position = canCoder_.GetPosition().GetValue();
    int deviceId = canCoder_.GetDeviceID(); // Obtain the CANCoder device ID
    std::string deviceIdStr = std::to_string(deviceId); // Convert the device ID to string

    position -= GetAlignment();

    // Convert to radians and back to use MathUtil AngleModulus
    units::degree_t normalizedPosition = frc::AngleModulus(position);
    return normalizedPosition;
}
