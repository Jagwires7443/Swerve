#pragma once

#include <string_view>

#include "infrastructure/SmartMotor.h"

namespace SparkMaxFactory
{
    // Print out string to index mapping (for updating switch/cases).
    void ConfigIndex() noexcept;

    // Create a SPARK MAX motor controller.
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

// Periodic Frame Periods (see SetPeriodicFramePeriod)
//   https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames
//   This is set on the SPARK MAX, but not persistently.  It could be handled
//   by introducing some kind of "volatile" flag in the config information that
//   would mean this had to be set reliably (there is no API to read this back,
//   and it would be just as efficient to simply set it, rather than checking
//   it periodically and reacting if it was found to be off).  These are useful
//   settings to be able to manage, so it is worth trying to do so.  If CAN bus
//   utilization is high, or if there needs to be more frequent adjustment by
//   any follower, or perhaps some other scenarios, it makes sense to be able
//   to control this setting.  The problem is power fail scenarios: to handle
//   these, the "HasReset" fault is used to determine when to reapply these
//   settings.  The roboRIO periodically recieves fault information in status
//   frame 0, so it is very cheap to obtain current fault information from each
//   Spark Max.  Other config parameters are maintained through the
//   [Config]Periodic() method.  Periodic frame periods (only) are maintained
//   through the Periodic() method, which is very lightweight.  It is concerned
//   with collecting status information, in order to be able to collect and
//   summarize the history of any problems dealing with the given controller.
//   The bottom line here is that these appear here as any other config
//   parameter, but they are implemented in a very different way.  One of the
//   things this code does is to hide these implementation details.

// Control Frame Period (see SetControlFramePeriodMs)
// CAN Timeout (see SetCANTimeout)
//   These are managed on the roboRIO side of things.  There is no attempt to
//   cover these, they are simply left at default values.  Note that this can
//   lead to loop time overrun warnings, especially if a motor controller
//   reboots.

// Motor Inverted (see GetInverted / SetInverted)
//   This is managed on the roboRIO side of things.  SmartMotor handles this by
//   passing a Boolean through the constructor.

// Firmware Version (see GetFirmwareVersion / GetFirmwareString)
//   This is read-only and only changes when the firmware has been manually
//   updated.  This is checked, and there is a psuedo config parameter to
//   specify the expected value.

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
// value).  If there were a generic get/set arbitrary config parameter API, the
// strategy would probably be to check everything.  As it stands, the idea is
// to only check config parameters that are explicitly listed via SetConfig()
// and AddConfig() and to seek to use RestoreFactoryDefaults() to set eveything
// else to default values.  If it is important that a particular parameter be
// set to the default value, listing it will result in it being checked.  The
// intent is that this should generally not be necessary.

// "Firmware Version" and "kIdleMode" are always checked, regardless of any
// SetConfig()/AddConfig() listings.  This is because the former is something
// which is only read, but which is important to highlight, while the latter is
// defaulted different from the factory default and so needs to be checked to
// ensure the desired default is persisted.

// kPolePairs
//   This configuration parameter also has no set and get and would only change
//   if the motor to be controlled were constructed differently than the NEO or
//   NEO550.  This parameter cannot be managed, and there is no reason to
//   manage it in this code.  If there is a motor with a differnt value for
//   this parameter, it will probably be made a persistent setting and be
//   managed via the REV Hardware Client (same as with Motor Type).

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
//   In alternate encoder mode, this parameter will be checked automatically,
//   because it is tied up this the other settings involved in constructing a
//   SparkMax.  This is done, even though it is not a managed config parameter.

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

// kLimitSwitchFwdPolarity: Forward; 0 = Normally Open, 1 = Normally Closed
// kLimitSwitchRevPolarity: Reverse; 0 = Normally Open, 1 = Normally Closed
// kHardLimitFwdEn
// kHardLimitRevEn
//   Limit switches (hard limits) only work when not in Alternate Encoder Mode.
//   For polarity, there are similar issues with get (none) and set (side
//   effect of calling GetForwardLimitSwitch() or GetReverseLimitSwitch()) as
//   with analog feedback.  Enable get and set are via SparkMaxLimitSwitch.  By
//   definition, there isn't much code to handle a hard limit switch, leaving
//   this mainly to configuration.  It may be useful to be able to report the
//   status, and to enable/disable the limits.  Changing the polarity via
//   config is fully supported, but the polarity is not a dynamic setting and
//   applying changes to these settings (forward/reverse polarity) only takes
//   effect once these settings have been "burned".

// kFollowerID
// kFollowerConfig
//   This one/two is a bit involved.  See
//   <https://docs.wpilib.org/en/stable/docs/software/can-devices/can-addressing.html>.
//
//   If these are both zero (the default), this controller is not a follower.
//   In code, these are handled via IsFollower() and Follow().  IsFollower() is
//   the only way to get anything, and all it indicates is default or not.  For
//   set, there are two forms of Follow(), but only the more general one is
//   needed here (the one which takes a "struct" ExternalFollower", and other
//   parameters).
//
//   To follow another Spark Max, kFollowerID is "0x02051800 | <other CAN ID>".
//   More generally, kFollowerID is the value to match in the FRC CAN header.
//   The 6 least-significant bits contain the CAN ID of the device to follow.
//   The REV "Configuration Parameters" page (linked above) describes
//   kFollowerConfig (although not super well).  The easiest way to arrive at
//   the proper values for these settings is to start with either
//   "kFollowerSparkMax" or "kFollowerPhoenix", from the "CANSparkMax.h" header
//   file.  Then, use binary or (|) to mix in the CAN ID and inversion bit, the
//   former into kFollowerID and the latter into kFollowerConfig (| 0x00004000)
//   -- this can be done as part of specifying the contstant config parameters.
//
//   This code treats followers as something to manage via config parameters,
//   so they resume after any sort of controller reset event.  With the
//   asymmetry in get and set, things have to be handled very carefully.  And
//   the Follow() function winds up taking some of the fields in the two config
//   parameters as additional parameters, so everything is somewhat convoluted.
//
//   On the get side, the code checks all the config parameters specified via
//   SetConfig()/GetConfig().  It makes three determinations; the first is if
//   there was some problem getting the paramter to check.  This can happen for
//   a few reasons, including listing an alternate encoder config parameter
//   when the normal encoder applies (or the other way around).  The follower
//   config is the only case where such a problem is expected in the absence of
//   some type of user error or unexected behavior (and only when non-default).
//   The remaining two checks are default/non-default, and matching the given
//   value or not.  For follower config, any non-default config is assumed to
//   be correct, when some non-default config has been specified.  So, if there
//   is a mismatch between two non-default settings, it will not be reported.
//   However, any non-default setting will be reapplied any time config
//   parameters are persisted.  On the set side, any problem is either a user
//   error or unexpected behavior, including for follower config.

//   The implication here is that it is possible to have problems updating from
//   one non-default setting to another (including changing the CAN ID to
//   follow).  Again, these settings will be updated any time config parameters
//   are being persisted.  The difference is that there will be no notification
//   that these changes needing to be persisted.  So, be sure to go through
//   this process following any such change.  Alternatively, the REV Hardware
//   Client may be used to make such changes, but this is problematic since the
//   listed config parameters will need to be updated regardless.

//   Finally, note that both parameters are set via a single call.  This causes
//   some slight additional complexity, but things are already complicated.

// At last, the "normal" configuration parameters!  These work as one would
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
//   controller is operating in Alternate Encoder Mode or not -- this code does
//   not ever attempt to use both encoders or to manage parameters for both):
//     *  GetAverageDepth()/SetAverageDepth()
//     *  GetMeasurementPeriod()/SetMeasurementPeriod() [for SampleDelta]
//     *  GetPositionConversionFactor()/SetPositionConversionFactor()
//     *  GetVelocityConversionFactor()/SetVelocityConversionFactor()
//   Note that the last four of these (only two apply) may be handled in this
//   class, rather than on the motor controller.  In this case, these are set
//   to unity (the default) and then managed to ensure they remain there.

// kCurrentChop
// kCurrentChopCycles
// kSmartCurrentStallLimit
// kSmartCurrentFreeLimit
// kSmartCurrentConfig
//   Managing these settings is problematic.  To start, there is no get.  So,
//   it would be difficult to manage these, short of just setting them in a
//   pathologic manner.  These settings are quite high by default, so raising
//   them is not needed.  But lowering them could make sense, especially when
//   controlling a NEO 550.  REV (rightly) cautions against bypassing the
//   protection these setting may provide, and provides data to help in working
//   out appropriate limits.  Also, on the set side, a single function sets
//   multiple config parameters.  This is all similar to issues discussed for
//   follower configuration.  Note that using closed-loop current control may
//   be an alternative to changing these defaults.

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
// kSmartMotionAccelStrategy_#; kTrapezoidal = 0, kSCurve = 1
//   These relate to closed-loop control.  The "_0" set is used for position,
//   and the "_1" set is used for velocity.  There is also a "_2" and "_3" set,
//   but these are not currently being used (or managed).  They could be useful
//   if multiple gear ratios are possible or in other scenarios, in the future.
//   In particular, closed-loop voltage/current control could use the
//   additional sets.
//   These are handled via GetPIDController(), plus one of the following:
//     *  GetP()/SetP()
//     *  GetI()/SetI()
//     *  GetD()/SetD()
//     *  GetFF()/SetFF()
//     *  GetIZone()/SetIZone()
//     *  GetIMaxAccum()/SetIMaxAccum()
//     *  GetDFilter()/SetDFilter()
//     *  GetOutputMin()/SetOutputRange()
//     *  GetOutputMax()/SetOutputRange()
//     *  GetSmartMotionMaxVelocity()/SetSmartMotionMaxVelocity()
//     *  GetSmartMotionMaxAccel()/SetSmartMotionMaxAccel()
//     *  GetSmartMotionMinOutputVelocity()/SetSmartMotionMinOutputVelocity()
//     *  GetSmartMotionAllowedClosedLoopError()/SetSmartMotionAllowedClosedLoopError()
//     *  GetSmartMotionAccelStrategy()/SetSmartMotionAccelStrategy()
// Note that kOutputMin_# and kOutputMax_# are linked by a common set function.
// It's a good idea to specify both of these if specifying either, but the code
// does handle this complication.

// These are the values expected following RestoreFactoryDefaults().  Note that
// kIdleMode is 0 (not 1) from the factory, but since it persists across this
// call, it is treated as having a default value of 1 in this code.  There is
// no reason to specify parameters which are to be set to these default values,
// as this will happen automatically.  These are listed to document the managed
// configuration parameters, for SetConfig()/AddConfig().  This covers the name
// used in this code, the type, and default value for each managed parameter.

// Also note that SmartMotionAccelStrategy does not appear to be implemented;
// if it is set to kSCurve, this succeeds -- but it reads back as kTrapezoidal.
namespace SparkMaxFactory
{
    // Version 1.5.2; all configuration parameters are current at this release.
    // **** DO NOT ALTER THIS LIST WITHOUT UNDERSTANDING THE IMPLICATIONS! ****
    // In particular, std::map is sorted, so the order here is meaningless.
    // The way forward (better readability/maintainability) is a constexpr map.
    const SmartMotorBase::ConfigMap configDefaults = {
        {"kStatus0", uint{10}}, // ms
        {"kStatus1", uint{20}}, // ms
        {"kStatus2", uint{50}}, // ms
        {"Firmware Version", uint{0x01050002}},
        {"kIdleMode", uint{1}},
        {"kLimitSwitchFwdPolarity", uint{0}},
        {"kLimitSwitchRevPolarity", uint{0}},
        {"kHardLimitFwdEn", bool{true}},
        {"kHardLimitRevEn", bool{true}},
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
        {"kEncoderSampleDelta", uint{100}}, // Actual default differs from doc.
        {"kAltEncoderSampleDelta", uint{100}},
        {"kPositionConversionFactor", double{1.0}},
        {"kAltEncoderPositionFactor", double{1.0}},
        {"kVelocityConversionFactor", double{1.0}},
        {"kAltEncoderVelocityFactor", double{1.0}},
        {"kCurrentChop", double{115.0}}, // Amps
        {"kCurrentChopCycles", uint{0}},
        {"kSmartCurrentStallLimit", uint{80}}, // Amps
        {"kSmartCurrentFreeLimit", uint{20}},  // Amps
        {"kSmartCurrentConfig", uint{10000}},  // RPM
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
        {"kSmartMotionAccelStrategy_0", uint{0}}, // Doc says this is double.
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
        {"kSmartMotionAccelStrategy_1", uint{0}},
    };
}
