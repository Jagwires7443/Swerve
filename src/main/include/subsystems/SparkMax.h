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
//   is the first of six parameters that persist across a call to
//   RestoreFactoryDefaults(), but there is no reason to manage it, or even
//   to ever manually set it (using the REV Hardware Client).

// kCanID
//   This can only be set using the REV Hardware Client.  It is possible to get
//   it via GetDeviceId(), but this is pointless, as it involves messaging with
//   the controller, using the CAN ID in question.  So, there is no reason to
//   manage it.  This is the second of six parameters that persist, and it is
//   the first of four that should be manually set and saved using the Hardware
//   Client, before attempting to do anything through this code.

// kMotorType; 0 = Brushed, 1 = Brushless
// kSensorType; 0 = No Sensor, 1 = Hall Sensor, 2 = Encoder
//   These configuration parameters have no explicit set and no get at all.  By
//   constructing a SparkMax, a rev::CANSparkMax is constructed, with the motor
//   and sensor types as they must be for a NEO or NEO550.  In other words, the
//   values for these configuration parameters are invariant in this context.
//   Without explicitly managing these, they will be set and saved.  There is
//   no reason to allow these settings to be altered and they cannot be
//   checked, so they do not appear in this list of managed configuration
//   parameters.  As reccommended by REV, these are the second and third of
//   four that should be manually set and saved using the Hardware Client.  And
//   these are the third and fourth of the six parameters that persist.

// kDataPortConfig; 0 = Default (limit switches are enabled), 1 = Alternate
//  Encoder Mode (limit switches are disabled / alternate encoder is enabled)
//   This configuration parameter also has no explicit set and no get at all.
//   By constructing a SparkMax, either a rev::SparkMaxRelativeEncoder or a
//   rev::SparkMaxAlternateEncoder is constructed, setting this as a side
//   effect.  This is otherwise the same as the two prior parameters.  This
//   is the fourth of the four parameters to be manually set and saved, and the
//   fifth of the six parameters that persist.

// kIdleMode;  0 = Coast, 1 = Brake
//   This configuration parameter may be set via SetIdleMode(), and checked via
//   GetIdleMode().  This is a managed configuration parameter.  It is also the
//   sixth of the six parameters that persist (across a call to
//   RestoreFactoryDefaults()).  This may change dynamically, but the default
//   is "Brake".  When a new Spark Max comes from the factory, this is actually
//   "Coast" but, because it persists, it will normally be "Brake", in this
//   context.  When managing settings, it important to lock out normal commands
//   to the controller.  This ensures this setting is not dynamically changed
//   during the process of saving updated configuration parameters.

// Again, use the REV HardWare Client to set the four manually managed config
// parameters.  First, be sure the firmware is current and update it as
// appropriate.  Then, specify the CAN ID followed by the motor type (which
// should result in the sensor type also being correctly specified).  Next,
// specify Alternate Encoder Mode (or turn this off); this is found under the
// "Advanced" tab.  Finally, permanently save these settings, via "Burn Flash".

// From here on, all configuration parameters are set to their given defaults
// by a call to RestoreFactoryDefaults().  There is no reason to ever set any
// of these manually, as they can be automatically managed via this code.  And,
// they may be returned to the default value, even if this code is not managing
// them (since it will call this function, as part of persisting config
// parametrs).  This applies to all parametrs below which show up only in
// comments (and not in the list of managed parameters with their default
// value).

// kPolePairs
//   This configuration parameter also has no set and get and would only change
//   if the motor to be controlled were constructed differently than the NEO or
//   NEO550.  This parameter cannot be managed, and there is no reason to
//   manage it in this code.  If there is a motor with a differnt value for
//   this parameter, it will probably be made a persistent setting and be
//   managed via the REV Hardware Client (same as with Motor Type).

// kCurrentChop
// kCurrentChopCycles
// kSmartCurrentStallLimit
// kSmartCurrentFreeLimit
// kSmartCurrentConfig
//   Right now, this code does not attempt to manage these settings.  For two
//   of these (kCurrentChop and kSmartCurrentStallLimit), there is no get.  So,
//   it would be difficult to manage these, short of periodically setting them.
//   These limits are set high, and REV cautions against bypassing protection
//   they provide.  So, this code intentionally does not attempt managing these
//   (and it's probably a good idea to leave these alone manually).  If these
//   are ever manually modified, this code will call RestoreFactoryDefaults()
//   if config parameters are ever persisted, causing them to return to default
//   values.  This code most likely should never attempt to manage these.

// kEncoderCountsPerRev
// kAltEncoderCountsPerRev
//   This is handled via either GetEncoder() or GetAlternateEncoder(), plus
//   GetCountsPerRevolution() to get these config parameters.  Setting these is
//   a side effect of GetEncoder() or GetAlternateEncoder().  One of these is
//   called as part of constructing a SparkMax, so these should be correct at
//   the point they are checked, unless there is a mismatch with the default,
//   and the controller has reset.  So, it is hard to detect a mismatch.  Also,
//   it is only possible to check the one of these which corresponds to the way
//   things were set up at the point the SparkMax was constructed.  And, it may
//   be the case that get is reading information in the roboRIO, not directly
//   from the controller.  For these reasons, this is not managed but will have
//   been set (based on the `encoderCount` parameter of SparkMax constructor)
//   any time configuration parameters are saved.  To top it all off, the
//   not-alternate encoder count likely only applies for a brushed motor.

// kAnalogPositionConversion
// kAnalogVelocityConversion
// kAnalogAverageDepth
// kAnalogSensorMode
// kAnalogInverted
// kAnalogSampleDelta
//   Right now, this code does not attempt to manage these settings.  For one
//   of these (kAnalogSensorMode), there is no get.  So, managing this would be
//   difficult, short of periodically setting it.  And, setting it is done as a
//   side effect of calling GetAnalog(), which is ordinarily only called once
//   when setting things up.  This code does not currently support the analog
//   feedback offered by Spark Max.  In most scenarios, there are reasons for
//   preferring an encoder, and this code is oriented around this type of
//   sensor instead.  As above, these will return to the default values any
//   time RestoreFactoryDefaults() is called (during saving of updated config
//   parameters).

// kLimitSwitchFwdPolarity
// kLimitSwitchRevPolarity
// kHardLimitFwdEn
// kHardLimitRevEn
//   Limit switches (hard limits) only work when not in Alternate Encoder Mode.
//   There are similar issues with get (none) and set (side effect of calling
//   GetForwardLimitSwitch() or GetReverseLimitSwitch()) as with analog
//   feedback.  Although it may make sense to add support in the future, this
//   code does not attempt to manage these settings right now.  The point about
//   these settings returning to default values applies (as for analog feedback
//   above).

// kFollowerID
// kFollowerConfig
//   These are handled via IsFollower() and Follow().  This code treats
//   followers as something to manage via config parameters, so they resume
//   after any sort of controller power event.  The get and set are asymmetric,
//   so the way things are handled is that the check simply treats the settings
//   as non-default if IsFollow() returns true.  If they are non-default, it is
//   assumed that they match any specified non-default value.  Therefore, it is
//   possible to have problems changing from one non-default setting to another
//   (including changing the CAN ID to follow).  To get around this, these will
//   be updated any time config parameters are being updated and saved.  So, be
//   sure to go through this process following any such change.
//   Alternatively, the REV Hardware Client may be used to make such changes.
//   See the Spark Max documentation to determine the proper values to specify.

// Finally, the "normal" configuration parameters!  These work as one would
// expect, with means of getting and setting them.  They are fully managed by
// this code.

// kSoftLimitFwd
// kSoftLimitRev
//   These are handled via GetSoftLimit() and SetSoftLimit().

// kRampRate
// kClosedLoopRampRate
//   These are handled via GetOpenLoopRampRate()/SetOpenLoopRampRate() or
//   GetClosedLoopRampRate()/SetClosedLoopRampRate().

// kCompensatedNominalVoltage
//   This is handled via GetVoltageCompensationNominalVoltage() and
//   EnableVoltageCompensation()/DisableVoltageCompensation().  Note that using
//   this may increase brownout issues, although it can provide more consistent
//   behavor.

// kAltEncoderInverted
//   This is handled via GetAlternateEncoder() plus GetInverted() and
//   SetInverted().  This only applies when operating in Alternate Encoder Mode
//   (when SparkMax has been created with a non-zero `encoderCount` parameter).

// kEncoderAverageDepth
// kAltEncoderAverageDepth
// kEncoderSampleDelta
// kAltEncoderSampleDelta
// kPositionConversionFactor
// kAltEncoderPositionFactor
// kVelocityConversionFactor
// kAltEncoderVelocityFactor
//   These are handled via either GetEncoder() or GetAlternateEncoder(), plus
//   one of the following (only one set of these applies, depending on if the
//   controller is operating in Alternate Encoder Mode or not):
//     *  GetAverageDepth()/SetAverageDepth()
//     *  GetMeasurementPeriod()/SetMeasurementPeriod() [for SampleDelta]
//     *  GetPositionConversionFactor()/SetPositionConversionFactor()
//     *  GetVelocityConversionFactor()/SetVelocityConversionFactor()

// kP_#
// kI_#
// kD_#
// kF_#
// kIZone_#
// kIMaxAccum_#
// kDFilter_#
// kOutputMin_#
// kOutputMax_#
// kSmartMotionMaxVelocity_#
// kSmartMotionMaxAccel_#
// kSmartMotionMinVelOutput_#
// kSmartMotionAllowedClosedLoopError_#
// kSmartMotionAccelStrategy_#
//   These relate to closed-loop control.  The "_0" set is used for position,
//   and the "_1" set is used for velocity.  There is also a "_2" and "_3" set,
//   but these are not currently being used (or managed).  They could be useful
//   if multiple gear ratios are possible or in other scenarios, in the future.
//   These are handled in one of two ways, either through the PID controller or
//   encoder.
//   Use GetPIDController(), plus one of the following:
//     *  GetP()/SetP()
//     *  GetI()/SetI()
//     *  GetD()/SetD()
//     *  GetFF()/SetFF()
//     *  GetIZone()/SetIZone()
//     *  GetDFilter()/SetDFilter()
//     *  GetOutputMin()/SetOutputRange()
//     *  GetOutputMax()/SetOutputRange()
//   Use GetEncoder()/GetAlternateEncoder(), plus one of the following:
//     *  GetIMaxAccum()/SetIMaxAccum()
//     *  GetSmartMotionMaxVelocity()/SetSmartMotionMaxVelocity()
//     *  GetSmartMotionMaxAccel()/SetSmartMotionMaxAccel()
//     *  GetSmartMotionMinOutputVelocity()/SetSmartMotionMinOutputVelocity()
//     *  GetSmartMotionAllowedClosedLoopError()/SetSmartMotionAllowedClosedLoopError()
//     *  GetSmartMotionAccelStrategy()/SetSmartMotionAccelStrategy()

// These are the values expected following RestoreFactoryDefaults().  Note that
// kIdleMode is 0 (not 1) from the factory, but since it persists across this
// call, it is treated as having a default value of 1 in this code.  There is
// no reason to specify parameters which are to be set to these default values,
// as this will happen automatically.  These are listed to document the managed
// configuration parameters, for SetConfig()/AddConfig().  This covers the name
// used in this code, the type, and default value for each managed parameter.
namespace SparkMaxFactory
{
    // Version 1.5.2; all configuration parameters are current at this release.
    const SmartMotorBase::ConfigMap configDefaults = {
        {"Firmware Version", uint{0x01050002}},
        {"kIdleMode", uint{1}},
        {"kFollowerID", uint{0}},
        {"kFollowerConfig", uint{0}},
        {"kSoftLimitFwd", double{0.0}},
        {"kSoftLimitRev", double{0.0}},
        {"kRampRate", double{0.0}},
        {"kClosedLoopRampRate", double{0.0}},
        {"kCompensatedNominalVoltage", double{0.0}},
        {"kAltEncoderInverted", bool{false}},
        {"kEncoderAverageDepth", uint{64}},
        {"kAltEncoderAverageDepth", uint{64}},
        {"kEncoderSampleDelta", uint{200}},
        {"kAltEncoderSampleDelta", uint{200}},
        {"kPositionConversionFactor", double{1.0}},
        {"kAltEncoderPositionFactor", double{1.0}},
        {"kVelocityConversionFactor", double{1.0}},
        {"kAltEncoderVelocityFactor", double{1.0}},
        {"kP_0", double{0.0}},
        {"kI_0", double{0.0}},
        {"kD_0", double{0.0}},
        {"kF_0", double{0.0}},
        {"kIZone_0", double{0.0}},
        {"kIMaxAccum_0", double{0.0}},
        {"kDFilter_0", double{0.0}},
        {"kOutputMin_0", double{-1.0}},
        {"kOutputMax_0", double{1.0}},
        {"kSmartMotionMaxVelocity_0", double{0.0}},
        {"kSmartMotionMaxAccel_0", double{0.0}},
        {"kSmartMotionMinVelOutput_0", double{0.0}},
        {"kSmartMotionAllowedClosedLoopError_0", double{0.0}},
        {"kSmartMotionAccelStrategy_0", double{0.0}},
        {"kP_1", double{0.0}},
        {"kI_1", double{0.0}},
        {"kD_1", double{0.0}},
        {"kF_1", double{0.0}},
        {"kIZone_1", double{0.0}},
        {"kIMaxAccum_1", double{0.0}},
        {"kDFilter_1", double{0.0}},
        {"kOutputMin_1", double{-1.0}},
        {"kOutputMax_1", double{1.0}},
        {"kSmartMotionMaxVelocity_1", double{0.0}},
        {"kSmartMotionMaxAccel_1", double{0.0}},
        {"kSmartMotionMinVelOutput_1", double{0.0}},
        {"kSmartMotionAllowedClosedLoopError_1", double{0.0}},
        {"kSmartMotionAccelStrategy_1", double{0.0}},
    };
}
