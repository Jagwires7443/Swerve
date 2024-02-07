#include "infrastructure/PWMAngleSensor.h"

#include "infrastructure/ShuffleboardWidgets.h"
#include <iostream>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

AngleSensor::AngleSensor(int deviceID, units::turn_t alignment) noexcept
    : canCoder_(deviceID) {
    // Configure the sensor range and other settings
    canCoder_.GetConfigurator().Apply(
        ctre::phoenix6::configs::MagnetSensorConfigs()
            .WithAbsoluteSensorRange(ctre::phoenix6::signals::AbsoluteSensorRangeValue::Signed_PlusMinusHalf)
            .WithMagnetOffset(alignment.value())
            // .WithSensorDirection(ctre::phoenix6::signals::SensorDirectionValue::Clockwise_Positive)
    );
     frc::SmartDashboard::PutNumber("Alignment", alignment.value());
    // Other configuration as needed...
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
/*
std::optional<units::angle::degree_t> AngleSensor::GetAbsolutePosition() noexcept {
    units::degree_t position = canCoder_.GetPosition().GetValue();
    // Position will already within [-180 to 180) degree range, offset to match magnet position
    frc::SmartDashboard::PutNumber("AngleSensor::GetAbsolutePosition Value", position.value());
    return position;
}
*/
std::optional<units::angle::degree_t> AngleSensor::GetAbsolutePosition() noexcept {
    units::degree_t position = canCoder_.GetPosition().GetValue();  // Assuming this returns units::angle::degree_t
    frc::SmartDashboard::PutNumber("AngleSensor::GetAbsolutePosition Value", position.value());

    double normalizedPosition = std::fmod((position.value() / 360) + 360.0, 360.0);  // Normalize to [0, 360) range

    // If normalizedPosition is greater than 180, adjust to [-180, 180) range
    if (normalizedPosition > 180.0) {
        normalizedPosition -= 360.0;
    }

    return units::angle::degree_t(normalizedPosition);  // Construct a new degree_t with the normalized value
}


std::optional<units::angle::turn_t> AngleSensor::GetAbsolutePositionWithoutAlignment() noexcept
{
    units::degree_t position = canCoder_.GetPosition().GetValue();

    position -= GetAlignment();

    return position;
}
