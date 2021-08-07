#pragma once

#include <frc/DigitalInput.h>
#include <frc/DutyCycle.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/shuffleboard/ComplexWidget.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/smartdashboard/Sendable.h>
#include <frc/smartdashboard/SendableBuilder.h>
#include <frc/smartdashboard/SendableHelper.h>
#include <rev/CANEncoder.h>
#include <rev/CANPIDController.h>
#include <rev/CANSparkMax.h>
#include <units/angle.h>
#include <units/length.h>
#include <units/velocity.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// Some SPARK MAX settings must be set (and saved) in advance, via "REV
// Hardware Client":
//   *Drive and Turning Controllers*
//     * kCanID -- Unique ID for each SPARK MAX
//     * kMotorType/kSensorType -- BRUSHLESS/HALL_EFFECT
//
//   *Drive Controller*
//     * kDataPortConfig -- 0 (Default Mode, Not Used)
//     * kIdleMode -- IDLE_COAST
//
//   *Turning Controller*
//     * kDataPortConfig -- 1 (Alternate Encoder Mode)
//     * kIdleMode -- IDLE_BRAKE
// Manually configuring and saving these is the first step in commissioning a
// robot.  The second step is ensuring all the inversion constants are correct.

// Note that REV advises setting and saving kDataPortConfig *before* connecting
// things!  Also note that REV advises ensuring any/all inversions are set up
// before running any closed loop control on the turning motor controller.
// It's also a good idea to ensure all modules are properly aligned before
// any attempt to drive (including to work out PID settings).  Test mode is
// used to do everything after manually setting the minimal configuration
// (above) and taking care of inversions.

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
// changed.

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
// position on each swerve module.

// At this point, the focus shifts to using test mode of the drive subsystem to
// tune PID parameters; once these are worked out, return to test mode of the
// swerve modules to save this additional configuration.  This is the fith (and
// final) step in commissioning a robot, at least as the swerve modules go.

class SwerveModule
{
public:
  // The only ctor of the SwerveModule class.
  SwerveModule(
      const char *const name,
      const int driveMotorCanID,
      const int turningMotorCanID,
      const int turningEncoderPort,
      const int alignmentOffset) noexcept;

  SwerveModule(const SwerveModule &) = delete;
  SwerveModule &operator=(const SwerveModule &) = delete;

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

  // Drive is normally oriented around velocity, but distance enables odometry,
  // simple dead reckoning, etc.  Possibly useful for autonomous driving.

  // Sense.  Return the cumulative drive distance since the last reset.
  units::length::meter_t GetDriveDistance() noexcept;

  // Act.  Command the swerve module to drive for the specified distance.
  void SetDriveDistance(units::length::meter_t distance) noexcept;

  // Sense. Return the current drive velocity.
  units::velocity::meters_per_second_t GetDriveVelocity() noexcept;

  // Act.  Command the swerve module to accelerate to the specified velocity.
  void SetDriveVelocity(units::velocity::meters_per_second_t velocity) noexcept;

  // Commands which combine the above functions, useful for teleop driving.
  void ResetEncoders() noexcept;
  const frc::SwerveModuleState GetState() noexcept;
  void SetDesiredState(const frc::SwerveModuleState &state) noexcept;

  // Fancy test mode!
  void TestInit() noexcept;
  void TestPeriodic() noexcept;

  // Provide PID settings used for all motion control (besides from test mode).
  void TurningPositionPID(double P, double I, double IZ, double IM, double D, double DF, double F) noexcept;
  void DrivePositionPID(double P, double I, double IZ, double IM, double D, double DF, double F) noexcept;
  void DriveVelocityPID(double P, double I, double IZ, double IM, double D, double DF, double F) noexcept;

  // Need to derive from abstract Sendable class in order to be able to use the
  // Gyro UI element in Shuffleboard; note that this doesn't actually derive from
  // frc::Gyro or frc::GyroBase -- it's all down to inheritance and properties.
  class HeadingGyro : public frc::Sendable, public frc::SendableHelper<HeadingGyro>
  {
  public:
    HeadingGyro() noexcept {}

    HeadingGyro(const HeadingGyro &) = delete;
    HeadingGyro &operator=(const HeadingGyro &) = delete;

    void InitSendable(frc::SendableBuilder &builder)
    {
      builder.SetSmartDashboardType("Gyro");
      builder.AddDoubleProperty(
          "Value", [&]() -> double { return m_value; }, nullptr);
    }

    void Set(const double &value) { m_value = value; }

  private:
    double m_value = 0.0;
  };

private:
  int GetAbsolutePosition(const int frequency, const double output) noexcept;
  int GetAbsolutePosition() noexcept;

  void DoSafeTurningMotor(const char *const what, std::function<void()> work) noexcept;
  bool DidSafeTurningMotor(const char *const what, std::function<bool()> work) noexcept;
  void DoSafeDriveMotor(const char *const what, std::function<void()> work) noexcept;
  bool DidSafeDriveMotor(const char *const what, std::function<bool()> work) noexcept;

  void ConstructTurningMotor() noexcept;
  void ConstructDriveMotor() noexcept;

  void SetTurningPositionPID() noexcept;
  void SetDrivePositionPID() noexcept;
  void SetDriveVelocityPID() noexcept;

  bool VerifyTurningMotorControllerConfig() noexcept;
  bool VerifyDriveMotorControllerConfig() noexcept;

  void CreateTurningMotorControllerConfig() noexcept;
  void CreateDriveMotorControllerConfig() noexcept;

  const std::string m_name;
  const int m_driveMotorCanID;
  const int m_turningMotorCanID;
  int m_alignmentOffset = 0;
  bool m_driveMotorBrake = false;

  // These are set based on the mechanical and electrical construction of the
  // robot, and are never expected to change.
  const bool m_driveMotorInverted = false;
  const bool m_turningMotorInverted = false;
  const bool m_turningEncoderInverted = true;

  // Turning position PID
  double m_turningPosition_P = 0.0;
  double m_turningPosition_I = 0.0;
  double m_turningPosition_IZ = 0.0;
  double m_turningPosition_IM = 0.0;
  double m_turningPosition_D = 0.0;
  double m_turningPosition_DF = 0.0;
  double m_turningPosition_F = 0.0;

  // Drive position PID
  double m_drivePosition_P = 0.0;
  double m_drivePosition_I = 0.0;
  double m_drivePosition_IZ = 0.0;
  double m_drivePosition_IM = 0.0;
  double m_drivePosition_D = 0.0;
  double m_drivePosition_DF = 0.0;
  double m_drivePosition_F = 0.0;

  // Drive velocity PID
  double m_driveVelocity_P = 0.0;
  double m_driveVelocity_I = 0.0;
  double m_driveVelocity_IZ = 0.0;
  double m_driveVelocity_IM = 0.0;
  double m_driveVelocity_D = 0.0;
  double m_driveVelocity_DF = 0.0;
  double m_driveVelocity_F = 0.0;

  std::unique_ptr<frc::DigitalInput> m_turningPositionSource;
  std::unique_ptr<frc::DutyCycle> m_turningPosition;

  std::unique_ptr<rev::CANSparkMax> m_turningMotor;
  std::unique_ptr<rev::CANEncoder> m_turningEncoder;
  std::unique_ptr<rev::CANPIDController> m_turningPID;

  std::unique_ptr<rev::CANSparkMax> m_driveMotor;
  std::unique_ptr<rev::CANEncoder> m_driveEncoder;
  std::unique_ptr<rev::CANPIDController> m_drivePID;

  std::chrono::steady_clock::time_point m_verifyMotorControllersWhen;
  bool m_turningMotorControllerValidated = false;
  bool m_driveMotorControllerValidated = false;
  std::string m_turningMotorControllerConfig;
  std::string m_driveMotorControllerConfig;

  // Test Mode (only) instance of a "Gyro", needed for Shuffleboard UI.
  HeadingGyro m_headingGyro;

  // Test Mode (only) data, obtained but not owned.
  frc::SimpleWidget *m_turningPositionStatus = nullptr;
  frc::SimpleWidget *m_turningPositionFrequency = nullptr;
  frc::SimpleWidget *m_turningPositionOutput = nullptr;
  frc::SimpleWidget *m_turningPositionAlignment = nullptr;
  frc::SimpleWidget *m_turningPositionPosition = nullptr;
  frc::ComplexWidget *m_turningPositionHeading = nullptr;

  frc::SimpleWidget *m_turningMotorStatus = nullptr;
  frc::SimpleWidget *m_turningMotorSpeed = nullptr;
  frc::SimpleWidget *m_turningMotorPercent = nullptr;
  frc::SimpleWidget *m_turningMotorVoltage = nullptr;
  frc::SimpleWidget *m_turningMotorCurrent = nullptr;
  frc::SimpleWidget *m_turningMotorTemperature = nullptr;
  frc::SimpleWidget *m_turningMotorFaults = nullptr;
  frc::SimpleWidget *m_turningMotorStickyFaults = nullptr;
  frc::SimpleWidget *m_turningMotorDistance = nullptr;
  frc::SimpleWidget *m_turningMotorVelocity = nullptr;
  frc::SimpleWidget *m_turningMotorControl = nullptr;
  frc::SimpleWidget *m_turningMotorReset = nullptr;

  frc::SimpleWidget *m_driveMotorStatus = nullptr;
  frc::SimpleWidget *m_driveMotorSpeed = nullptr;
  frc::SimpleWidget *m_driveMotorPercent = nullptr;
  frc::SimpleWidget *m_driveMotorVoltage = nullptr;
  frc::SimpleWidget *m_driveMotorCurrent = nullptr;
  frc::SimpleWidget *m_driveMotorTemperature = nullptr;
  frc::SimpleWidget *m_driveMotorFaults = nullptr;
  frc::SimpleWidget *m_driveMotorStickyFaults = nullptr;
  frc::SimpleWidget *m_driveMotorDistance = nullptr;
  frc::SimpleWidget *m_driveMotorVelocity = nullptr;
  frc::SimpleWidget *m_driveMotorControl = nullptr;
  frc::SimpleWidget *m_driveMotorReset = nullptr;
};
