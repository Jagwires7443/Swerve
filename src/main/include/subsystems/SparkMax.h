#pragma once

#include <string_view>

#include "subsystems/SmartMotor.h"

namespace SparkMaxFactory
{
    std::unique_ptr<SmartMotorBase> CreateSparkMax(const std::string_view name, const int canId, const bool inverted, const int encoderCounts = 0) noexcept;
}

// Configuration:

// The basic approach here is to only manage settings which relate to
// functionality which is exposed through the current SmartMotor API; adding to
// this is straightforward.  However, this starts out coming at things from the
// full set of (in some cases, not so well) documented configuration settings
// that are found at this link:
//   <https://github.com/REVrobotics/SPARK-MAX-Documentation/blob
//   /f4f60b3f1b889f2e3727c4140a2a5e9fd7936262/software-resources
//   /configuration-parameters.md>.

// This is also informed by the "REV Hardware Client" GUI.  The full list
// follows but, first, here is a list of settings which do not have
// any configuration parameter:

// Motor Inverted (see GetInverted / SetInverted)
//   This is managed on the roboRIO side of things.  SmartMotor handles this by
//   passing a Boolean through the constructor.

// Firmware Version (see GetFirmwareVersion / GetFirmwareString)
//   This is read-only and only changes when the firmware has been manually
//   updated.  This is checked, and there is a psuedo config parameter to
//   specify the expected value.

// Periodic Frame Periods (see SetPeriodicFramePeriod)
//   This is set on the SPARK MAX, but not persistently.  It could be handled
//   by introducing some kind of "volatile" flag in the config information that
//   would mean this had to be set reliably (there is no API to read this back,
//   and it would be just as efficient to simply set it, rather than checking
//   it periodically and reacting if it was found to be off).  For now there is
//   no attempt to cover this, it is simply left at the default values.

// Control Frame Period (see SetControlFramePeriodMs)
// CAN Timeout (see SetCANTimeout)
//   These are managed on the roboRIO side of things.  There is no attempt to
//   cover these, it is simply left at the default value.

// Now, the list of actual Spark Max configuaration parameters:

// Reserved
// kSlot3Placeholder*
//   Currently, there are 24 "Reserved" and 12 "kSlot3Placeholder*"
//   configuration parameters.  These are not listed, for the obvious reason.

// kInputMode
// kCtrlType
//   These two configuration parameters are read-only (they can't actually be
//   configured); there is no reason to attempt to manage them.

// kInputDeadband
//   This parameter only applies for PWM control, and so is irrelevant.  This
//   is one of six parameters that persist across a call to
//   RestoreFactoryDefaults(), but there is no reason to manage it.

// kCanID
// kMotorType
// kSensorType
// kDataPortConfig

// kIdleMode

SmartMotorBase::ConfigMap configDefaults = {
    // Version 1.5.2; all configuration parameters are current at this release.
    {"Firmware Version", uint{0x01050002}},
};
