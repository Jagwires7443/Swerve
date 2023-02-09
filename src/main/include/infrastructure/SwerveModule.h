#pragma once

#include "Constants.h"
#include "infrastructure/PWMAngleSensor.h"
#include "infrastructure/ShuffleboardWidgets.h"
#include "infrastructure/SmartMotor.h"
#include "infrastructure/SparkMax.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/DataLog.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableHelper.h>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

// Some SPARK MAX settings must be set (and saved) in advance, via "REV
// Hardware Client":
//   *Drive and Turning Controllers*
//     * kCanID -- Unique ID for each SPARK MAX
//     * kMotorType/kSensorType -- BRUSHLESS/HALL_EFFECT
//     * kIdleMode -- IDLE_BRAKE
//     * kDataPortConfig -- 0 (Default Mode, Not Used)
// Manually configuring and saving these is the first step in commissioning a
// robot.  The second step is ensuring all the inversion constants are correct.

// Note that REV advises setting and saving kDataPortConfig *before* connecting
// things!  Also note that REV advises ensuring any/all inversions are set up
// before running any closed loop control on the turning motor controller.
// It's also a good idea to ensure all modules are properly aligned before
// any attempt to drive (including to work out PID settings).  Test mode is
// used to do everything after manually setting the minimal configuration
// (above) and taking care of inversions.  Specifically, the per-module
// Shuffleboard tabs (one for each corner) support this part of bring-up.

// When things are properly specified, positive values for "Control" in the
// test mode dashboard will cause "Position" to advance and "Percent" and
// "Velocity" to be positive; the converse holds for applying negative values
// for "Control".  Do not attempt to use PID control until all of this has been
// empirically verified for every robot/module!  There is no need to invert the
// drive motor (since forward is arbitrary on a swerve module); however, it
// makes sense to zero turning when the drive motor is facing "forward".  Use
// inversion of the turning motor to ensure that the absolute encoder reading
// advances when positive values for "Control" are applied to the turning motor
// (in addition to the checks specified above for both motors).  Finally, use
// inversion of the turning encoder to ensure "Distance" is also advancing.

// The strategy for managing motor controller settings is to ensure that proper
// settings are saved on the controller.  This way, should a controller reboot,
// it will come back properly configured.  Test mode provides a solid mechanism
// to check the settings, including the expected firmware version.  And, it
// provides a way to save the proper settings, including updating PID
// parameters.  Before settings are updated, RestoreFactoryDefaults() is called
// to ensure things start out with all unmanaged setting at the default values.

// To save motor controller configuration, bring up test mode, enable the
// robot, and wait for the information to populate.  At this stage, there is no
// reason to worry about PID settings.  Hit "Reset" for each motor controller
// to update and save the motor controller configuration.  Saving the
// configuration for each motor controller is the third step in commissioning a
// robot.  Note that this will also need to be done after any PID setting are
// changed, so changing a PID setting is a way to force a configuration update.

// Now, the alignment values can be measured and adjusted in the code; there is
// no saving these in the motor controllers, only in the constants used in this
// code.  Hitting "Reset" sets the alignment offset so that the current
// position becomes the zero position.  This is a good way to find a coarse
// alignment.  Do this, then manually turn the wheels to face 0, 90, 180, and
// 270 degrees.  At each position, verify these angles as closely as possible
// (for example, consider placing a straight-edge, across adjacent pairs to
// align them).  Then note the values of "Alignment", and of "Position" at each
// of the specified angles.  Now, use this information to fine-tune the
// alignment offset for each swerve module.  This is the fourth step in
// commissioning a robot; make these changes in the code and recheck the zero
// position on each swerve module.  ("Reset" clears any sticky errors as well.)

// There are also some constants to be set depending on the geometry of the
// robot, and on the empirically measured maximum drive speed.  To obtain the
// latter, support the robot so the drive wheels are free to spin.  Then, use
// "Control" to run each drive wheel at full speed in both directions, noting
// "Velocity" each way.  This will yield eight values; use the one with the
// smallest magnitude for this speed, dropping the sign if negative.  This is
// reported in units of revolutions per second.

// At this point, the focus shifts to using test mode of the drive subsystem to
// tune PID parameters; once these are worked out, return to test mode of the
// swerve modules to save this additional configuration.  This is the fith (and
// final) step in commissioning a robot, at least as the swerve modules go.

// The first set of PID parameters to tune is that for turning position.  There
// are several test mode routines meant to help here.  "Turning Max" is an open
// loop routine designed to allow the maximum angular velocity and acceleration
// for the turning modules to be measured.  The kTurningPositionMaxVelocity and
// kTurningPositionMaxAcceleration constants hold these values.  Start with the
// value set to unity (one) for both of these and bring up the PID Tuning graph
// to see the plots of these values.  This is done by selecting the "Enabled"
// control for the corresponding "PID Settings" panel.  Update these constants.

// Once this is done, the "Xs and Os" routine may be used to command turning at
// a regular interval.  It is useful to provide a simple feedforward ("F").
// Start with everything set to zero and find a value low enough that none of
// the modules turn, but reasonably close to the highest such value.  Then,
// slowly increase "P" to get a good response, with no oscillation and ensuring
// that it works well for all modules.  Do all of this with the modules up in
// the air.  It may be worth verifying everything on the ground, with a similar
// surface and weight as when on the field.  It should be possible to tune only
// "F" and "P" and get good results.  If not, consult any of the FRC-friendly
// PID tuning guides.

// Once the modules turn well, use the "Zero" routine to verify home position
// of all the modules.  Then, use the "Point" routine to verify that operator
// controls and the software is all working as expected.  Before moving on, the
// turning behavior should be operating well.

class SwerveModule
{
public:
  enum class GraphSelection
  {
    kNone = 0,
    kTurningRotation,
    kDrivePosition,
    kDriveVelocity,
  };

  // The only ctor of the SwerveModule class.
  SwerveModule(
      const char *const name,
      const int driveMotorCanID,
      const int turningMotorCanID,
      const int turningEncoderPort,
      const int alignmentOffset) noexcept;

  // No copy/assign.
  SwerveModule(const SwerveModule &) = delete;
  SwerveModule &operator=(const SwerveModule &) = delete;

  // This method is needed in order to calculate and apply PID output for
  // turning, when this control is being handled in this code (rather than on
  // the motor controller, which doesn't work at present due to a REV issue).
  // This also provides a means to alter the drive motor control based on the
  // commanded and actual turning position.  It provides a guaranteed chance to
  // read sensors, ahead of the next cycle of possible commands.  If the motors
  // are being directly controlled via SwerveModule() Test Mode (open-loop,
  // manual control from a per-module tab), the motors will not actually be
  // commanded.
  void Periodic() noexcept;

  // Is swerve module healthy and motor controller configuration current?
  // Note that the latter condition is only checked in test mode.
  bool GetStatus() const noexcept;

  // Set motor controller turning position using absolute position sensor (best
  // done when robot is at rest).
  void ResetTurning() noexcept;

  // Reset motor controller drive position/velocity to zero (best done when
  // robot is at rest).
  void ResetDrive() noexcept;

  // Sense.  Return the current absolute heading for the swerve module.
  units::angle::degree_t GetTurningPosition() noexcept;

  // Act.  Command the swerve module to take on the specified absolute heading.
  void SetTurningPosition(const units::angle::degree_t position) noexcept;

  // Determine if commanded turning position has been achieved, to within
  // specified tolerance.
  bool CheckTurningPosition(const units::angle::degree_t tolerance = 2.5_deg) noexcept;

  // Drive is normally oriented around velocity, but distance enables odometry,
  // simple dead reckoning, etc.  Possibly useful for autonomous driving.

  // Control brake/coast; the initial setting is brake mode off (coast mode).
  // Breaking is only applied when this mode has been enabled, when the turning
  // position has been reached to within the default tolerance of
  // CheckTurningPosition(), and when the commanded velocity is zero.  This is
  // only for velocity control -- under distance control brake mode is always
  // enabled (but the turning position is still factored in).
  void SetDriveBrakeMode(bool brake) noexcept { m_commandedBrake = brake; }

  // Sense.  Return the cumulative drive distance since the last reset.
  units::length::meter_t GetDriveDistance() noexcept;

  // Act.  Command the swerve module to drive for the specified distance.  Call
  // ResetDrive() to zero the distance, or supply a running total distance.
  void SetDriveDistance(units::length::meter_t distance) noexcept;

  // Determine if commanded drive distance has been achieved, to within
  // specified tolerance.
  bool CheckDriveDistance(const units::length::meter_t tolerance = 5_cm) noexcept;

  // Sense. Return the current drive velocity.
  units::velocity::meters_per_second_t GetDriveVelocity() noexcept;

  // Act.  Command the swerve module to accelerate to the specified velocity.
  void SetDriveVelocity(units::velocity::meters_per_second_t velocity) noexcept;

  // Commands which combine the above functions, useful for teleop driving.
  void ResetEncoders() noexcept;
  const frc::SwerveModuleState GetState() noexcept;
  const frc::SwerveModulePosition GetPosition() noexcept;
  void SetDesiredState(const frc::SwerveModuleState &state) noexcept;

  // Fancy test mode!  TestModeControl(true) puts the swerve modules in
  // low-level, open-lop, manual control.
  void TestModeControl(const bool enabled, const double turningVoltage, double driveVoltage) noexcept
  {
    m_testModeControl = enabled;

    if (m_testModeControl)
    {
      m_testModeTurningVoltage = 0.0;
      m_testModeDriveVoltage = 0.0;
    }
    else
    {
      m_testModeTurningVoltage = turningVoltage;
      m_testModeDriveVoltage = driveVoltage;
    }
  }

  std::tuple<double, double, double, double> TestModeGraphData(const GraphSelection graphSelection) noexcept
  {
    if (graphSelection != m_graphSelection)
    {
      m_graphSelection = graphSelection;
      SetStatusFramePeriods(m_graphSelection);
    }

    return std::make_tuple(m_processVariable, m_processError, m_processFirstDerivative, m_processSecondDerivitive);
  }

  void TestInit() noexcept;
  void TestExit() noexcept;
  void TestPeriodic() noexcept;

  // Provide PID settings used for all motion control (besides from test mode).
  void TurningPositionPID(double P, double I, double IZ, double IM, double D, double DF, double F, double V, double A) noexcept;
  void DrivePositionPID(double P, double I, double IZ, double IM, double D, double DF, double F, double V, double A) noexcept;
  void DriveVelocityPID(double P, double I, double IZ, double IM, double D, double DF, double F, double V, double A) noexcept;

  void BurnConfig() noexcept;
  void ClearFaults() noexcept;

private:
  void SetTurningPositionPID() noexcept;
  void SetDrivePositionPID() noexcept;
  void SetDriveVelocityPID() noexcept;
  void SetStatusFramePeriods(GraphSelection graphSelection) noexcept;

  const std::string m_name;

  // These are set based on the mechanical and electrical construction of the
  // robot, and are never expected to change.
  const bool m_driveMotorInverted{physical::kDriveMotorInverted};
  const bool m_turningMotorInverted{physical::kTurningMotorInverted};

  // Used for printf-style logging.
  wpi::log::StringLogEntry m_stringLog;

  // Until REV bug for continuous rotation is fixed, must do turning PID on the
  // roboRIO.  Setting m_rio false allows testing turning PID on the SPARK MAX.
  const bool m_rio{true};
  bool m_brakeApplied{true};
  double m_rioPID_F{pidf::kTurningPositionF};
  std::unique_ptr<frc::ProfiledPIDController<units::angle::degrees>> m_rioPIDController;

  // Turning position PID
  double m_turningPosition_P{pidf::kTurningPositionP};
  double m_turningPosition_I{pidf::kTurningPositionI};
  double m_turningPosition_IZ{pidf::kTurningPositionIZ};
  double m_turningPosition_IM{pidf::kTurningPositionIM};
  double m_turningPosition_D{pidf::kTurningPositionD};
  double m_turningPosition_DF{pidf::kTurningPositionDF};
  double m_turningPosition_F{pidf::kTurningPositionF};
  double m_turningPosition_V{pidf::kTurningPositionMaxVelocity};
  double m_turningPosition_A{pidf::kTurningPositionMaxAcceleration};

  // Drive position PID
  double m_drivePosition_P{pidf::kDrivePositionP};
  double m_drivePosition_I{pidf::kDrivePositionI};
  double m_drivePosition_IZ{pidf::kDrivePositionIZ};
  double m_drivePosition_IM{pidf::kDrivePositionIM};
  double m_drivePosition_D{pidf::kDrivePositionD};
  double m_drivePosition_DF{pidf::kDrivePositionDF};
  double m_drivePosition_F{pidf::kDrivePositionF};
  double m_drivePosition_V{pidf::kDrivePositionMaxVelocity};
  double m_drivePosition_A{pidf::kDrivePositionMaxAcceleration};

  // Drive velocity PID
  double m_driveVelocity_P{pidf::kDriveVelocityP};
  double m_driveVelocity_I{pidf::kDriveVelocityI};
  double m_driveVelocity_IZ{pidf::kDriveVelocityIZ};
  double m_driveVelocity_IM{pidf::kDriveVelocityIM};
  double m_driveVelocity_D{pidf::kDriveVelocityD};
  double m_driveVelocity_DF{pidf::kDriveVelocityDF};
  double m_driveVelocity_F{pidf::kDriveVelocityF};
  double m_driveVelocity_V{pidf::kDriveVelocityMaxAcceleration};
  double m_driveVelocity_A{pidf::kDriveVelocityMaxJerk};

  std::unique_ptr<AngleSensor> m_turningPositionPWM;

  std::unique_ptr<SmartMotorBase> m_turningMotorBase;
  std::unique_ptr<SmartMotor<units::angle::degrees>> m_turningMotor;

  std::unique_ptr<SmartMotorBase> m_driveMotorBase;
  std::unique_ptr<SmartMotor<units::length::meters>> m_driveMotor;

  std::chrono::steady_clock::time_point m_verifyMotorControllersWhen;
  bool m_turningMotorControllerValidated{true};
  bool m_driveMotorControllerValidated{true};
  std::string m_turningMotorControllerConfig;
  std::string m_driveMotorControllerConfig;

  // Last sensed state, updated in Periodic().  These are quick to read, as
  // they depend only on local sensor input (PWM absolute position), or on data
  // which arrives over CAN, via periodic staus frames, from the SPARK MAXes.
  // However, there's no point in repeatedly performing the same processing, so
  // a local copy is kept for even quicker access.
  bool m_absoluteSensorGood{false};
  bool m_turningPositionAsCommanded{false};
  units::angle::degree_t m_turningPosition{0};

  // Last commanded turn heading and drive distance/speed.
  units::angle::degree_t m_commandedHeading{0};
  bool m_commandedBrake{true};
  bool m_distanceVelocityNot{false};
  units::length::meter_t m_commandedDistance{0};
  units::velocity::meters_per_second_t m_commandedVelocity{0};

  // Low-level test mode.
  bool m_testModeControl{false};
  double m_testModeTurningVoltage{0.0};
  double m_testModeDriveVoltage{0.0};

  // When graphing, track time-based derivatives.
  GraphSelection m_graphSelection{GraphSelection::kNone};
  uint64_t m_lastFPGATime{0};
  double m_processVariable{0.0};
  double m_processError{0.0};
  double m_processFirstDerivative{0.0};
  double m_processSecondDerivitive{0.0};

  double m_turningControlUI{0.0};
  bool m_turningResetUI{false};
  double m_driveControlUI{0.0};
  bool m_driveResetUI{false};
};
