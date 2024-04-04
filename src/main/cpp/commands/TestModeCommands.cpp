// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TestModeCommands.h"

#include <cmath>
#include <functional>

void ZeroCommand::Execute() noexcept { (void)m_subsystem->ZeroModules(); }

void MaxVAndATurningCommand::Initialize() noexcept
{
    m_iteration = 0;

    m_subsystem->TestModeTurningVoltage(0.0);
    m_subsystem->TestModeDriveVoltage(0.0);
}

void MaxVAndATurningCommand::End(bool interrupted) noexcept
{
    m_subsystem->TestModeTurningVoltage(0.0);
    m_subsystem->TestModeDriveVoltage(0.0);
}

void MaxVAndATurningCommand::Execute() noexcept
{
    // This is expected to run at ~20Hz.  So 100 iterations is ~5s.
    if (m_iteration < 50)
    {
        m_subsystem->TestModeTurningVoltage(-12.0);
    }
    else
    {
        m_subsystem->TestModeTurningVoltage(+12.0);
    }

    if (++m_iteration >= 100)
    {
        m_iteration = 0;
    }
}

void MaxVAndADriveCommand::Initialize() noexcept
{
    m_iteration = 0;

    m_subsystem->TestModeTurningVoltage(0.0);
    m_subsystem->TestModeDriveVoltage(0.0);
}

void MaxVAndADriveCommand::End(bool interrupted) noexcept
{
    m_subsystem->TestModeTurningVoltage(0.0);
    m_subsystem->TestModeDriveVoltage(0.0);
}

void MaxVAndADriveCommand::Execute() noexcept
{
    // This is expected to run at ~20Hz.  So 100 iterations is ~5s.
    if (m_iteration < 50)
    {
        m_subsystem->TestModeDriveVoltage(-12.0);
    }
    else
    {
        m_subsystem->TestModeDriveVoltage(+12.0);
    }

    if (++m_iteration >= 100)
    {
        m_iteration = 0;
    }
}

void XsAndOsCommand::Initialize() noexcept
{
    m_iteration = 0;

    m_subsystem->ResetDrive();
}

void XsAndOsCommand::End(bool interrupted) noexcept
{
    m_subsystem->ResetDrive();
}

void XsAndOsCommand::Execute() noexcept
{
    // This is expected to run at ~20Hz.  So 100 iterations is ~5s.
    if (m_iteration < 100)
    {
        (void)m_subsystem->SetLockWheelsX();
    }
    else
    {
        (void)m_subsystem->SetTurnInPlace();
    }

    if (++m_iteration >= 200)
    {
        m_iteration = 0;
    }
}

void RotateModulesCommand::Initialize() noexcept
{
    m_iteration = 0;

    m_subsystem->ResetDrive();
}

void RotateModulesCommand::End(bool interrupted) noexcept
{
    m_subsystem->ResetDrive();
}

void RotateModulesCommand::Execute() noexcept
{
    // This is expected to run at ~20Hz.  So 100 iterations is ~5s.
    if (!m_subsystem->SetTurningPosition(1_deg * m_iteration))
    {
        return;
    }

    if (++m_iteration >= 360)
    {
        m_iteration = 0;
    }
}

void SquareCommand::Initialize() noexcept
{
    m_side = 0;

    m_subsystem->ResetDrive();
}

void SquareCommand::End(bool interrupted) noexcept
{
    m_subsystem->ResetDrive();
}

void SquareCommand::Execute() noexcept
{
    // First, command turning to the appropriate angle and go no further, until
    // the wheels are oriented appropriately.
    if (!m_subsystem->SetTurningPosition(90_deg * m_side))
    {
        return;
    }

    // Now, command driving a fixed distance and go no further, until it is
    // reached.
    if (!m_subsystem->SetDriveDistance(0.5_m))
    {
        return;
    }

    // Finished driving a side; now, set things up to start the next side, on
    // the next iteration.  After the fourth side, also reset the side counter.
    m_subsystem->ResetDrive();

    if (++m_side >= 4)
    {
        m_side = 0;
    }
}

void SpirographCommand::Initialize() noexcept
{
    m_angle = 0;

    m_subsystem->ResetDrive();
}

void SpirographCommand::End(bool interrupted) noexcept
{
    m_subsystem->ResetDrive();
}

void SpirographCommand::Execute() noexcept
{
    // First, command turning to the appropriate angle and go no further, until
    // the wheels are oriented appropriately.
    if (!m_subsystem->SetTurningPosition(1_deg * m_angle))
    {
        return;
    }

    // Now, command driving a fixed distance and go no further, until it is
    // reached.
    if (!m_subsystem->SetDriveDistance(0.5_m))
    {
        return;
    }

    // Finished driving a side; now, set things up to start the next side, on
    // the next iteration.  After the fourth side, also reset the side counter.
    m_subsystem->ResetDrive();

    m_angle += 75;
    if (m_angle >= 360)
    {
        m_angle -= 360;
    }
}

void OrbitCommand::Execute() noexcept
{
    m_subsystem->Drive(0_mps, 0_mps, 10_deg_per_s, false, 1_m, 0_m);
}

void PirouetteCommand::Initialize() noexcept
{
    // Arrange the origin and orientation of the coordinate system, so that the
    // robot is on the unit circle, at (1,0) and facing positive X.  (The robot
    // does not move here, the coordinate system itself moves.)
    m_subsystem->ResetOdometry(frc::Pose2d(1_m, 0_m, frc::Rotation2d(0_deg)));
}

void PirouetteCommand::End(bool interrupted) noexcept
{
}

void PirouetteCommand::Execute() noexcept
{
    // Determine the direction the robot is facing, and where it is positioned.
    // As it happens, the direction the robot is facing is not used here.
    const frc::Pose2d pose = m_subsystem->GetPose();

    // Now, work out the angle of where the robot is positioned on the unit
    // circle, which is independent of where it is facing.  It may drift either
    // inside or outside of the unit circle.
    const double X = pose.X().to<double>();
    const double Y = pose.Y().to<double>();
    frc::Rotation2d major_angle(X, Y);

    // The robot should translate tangentially to the unit circle, in a
    // counter-clockwise fashion.  So, add 90 degrees.
    major_angle = major_angle + frc::Rotation2d(90_deg);

    // Work out the radius of the circle where robot is actually sitting, so it
    // may be used to make an adjustment to try to get back onto the unit
    // circle.  Positive error means robot is outside target circle, negative
    // means inside.  Outside correction is to increase the computed angle;
    // inside is to decrease.
    double major_error = std::hypot(X, Y) - 1.0;

    // Limit the maximum correction to apply and then apply it.
    if (major_error > 0.5)
    {
        major_error = 0.5;
    }
    else if (major_error < -0.5)
    {
        major_error = -0.5;
    }
    major_angle = major_angle + frc::Rotation2d(major_error * 20_deg);

    // Update X and Y drive command inputs based on above computations, while
    // always commanding rotation.  The commanded drive needs to be adjusted to
    // account for the actual orientation of the robot, thus field oriented
    // applies.
    m_subsystem->Drive(major_angle.Sin() * 1_mps, major_angle.Cos() * 1_mps, 10_deg_per_s, true);
}

void SpinCommand::Initialize() noexcept
{
    m_angle = 0;
    m_delay = 0;

    m_subsystem->ResetDrive();
}

void SpinCommand::End(bool interrupted) noexcept
{
    m_subsystem->ResetDrive();
}

void SpinCommand::Execute() noexcept
{
    if (!m_subsystem->SetTurnInPlace())
    {
        return;
    }

    if (!m_subsystem->SetTurnToAngle(1_deg * m_angle))
    {
        return;
    }

    // Reached desired angle; delay for 2.5s.
    if (++m_delay < 25)
    {
        return;
    }
    m_delay = 0;

    if (m_angle == 0)
    {
        m_angle = 90;
    }
    else if (m_angle == 90)
    {
        m_angle = 270;
    }
    else if (m_angle == 270)
    {
        m_angle = 0;
    }
}

void SysIdCommand::Initialize() noexcept
{
    m_routine = 0;
    m_mode = 0;
    m_stage = 0;

    m_subsystem->ResetDrive();

    auto driveDrive = [this](units::voltage::volt_t voltage) -> void
    {
        m_subsystem->TestModeDriveVoltage(voltage.value()); // XXX fix units for arg
    };

    auto driveLog = [this](frc::sysid::SysIdRoutineLog *logger) -> void
    {
        m_subsystem->SysIdLogDrive(logger);
    };

    m_driveSysId = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config(std::nullopt, std::nullopt, std::nullopt, std::nullopt),
        frc2::sysid::Mechanism(driveDrive, driveLog, m_subsystem, "Drive"));

    auto steerDrive = [this](units::voltage::volt_t voltage) -> void
    {
        m_subsystem->TestModeTurningVoltage(voltage.value()); // XXX fix units for arg
    };

    auto steerLog = [this](frc::sysid::SysIdRoutineLog *logger) -> void
    {
        m_subsystem->SysIdLogSteer(logger);
    };

    m_steerSysId = std::make_unique<frc2::sysid::SysIdRoutine>(
        frc2::sysid::Config(std::nullopt, std::nullopt, std::nullopt, std::nullopt),
        frc2::sysid::Mechanism(steerDrive, steerLog, m_subsystem, "Drive")); 
}

void SysIdCommand::End(bool interrupted) noexcept
{
    m_commandPtr = nullptr;
    m_driveSysId = nullptr;
    m_steerSysId = nullptr;

    m_subsystem->ResetDrive();

}

void SysIdCommand::Execute() noexcept
{
    // Sequentially characterize drive base (m_routine counter):
    //   0) Drive forward/reverse with all swerve modules in the same direction;
    //   1) Spin clockwise/counterclockwise with all swerve modules on circumference;
    //   2) Spin all individual swerve modules clockwise/counterclockwise.
    //   3) Done.
    // Three-layer iteration:
    //   1) Each individual mechanism (above);
    //   2) Two modes, each with two directions (m_mode counter);
    //   3) Prepare/Initialize/Execute/End (m_stage counter).

    // XXX FIX STATE MACHINE!

    // Outer iteration provides these two elements; increment it when both of
    // the inner iterations complete.
    std::function<bool()> prepare{nullptr};
    frc2::sysid::SysIdRoutine *routine{nullptr};

    switch (m_routine)
   {
     case 0:
        prepare = std::bind(&DriveSubsystem::ZeroModules, m_subsystem);
        routine = m_driveSysId.get();

        break;
    case 1:
        prepare = std::bind(&DriveSubsystem::SetTurnInPlace, m_subsystem);
        routine = m_driveSysId.get();

        break;
    case 2:
        prepare = std::bind(&DriveSubsystem::ZeroModules, m_subsystem);
        routine = m_steerSysId.get();

        break;
    case 3:
        return;
   }

   switch (m_stage)
    {
         case 0: // Prepare.
        if (prepare())
        {
            // Middle iteration provides this element; increment it when the single
            // inner iteration completes.
            switch (m_mode)
            {
            case 0:
                m_commandPtr = std::make_unique<frc2::CommandPtr>(routine->Quasistatic(frc2::sysid::Direction::kForward));
                m_mode = 1;

                break;
            case 1:
                m_commandPtr = std::make_unique<frc2::CommandPtr>(routine->Quasistatic(frc2::sysid::Direction::kReverse));
                m_mode = 2;

                break;
            case 2:
                m_commandPtr = std::make_unique<frc2::CommandPtr>(routine->Dynamic(frc2::sysid::Direction::kForward));
                m_mode = 3;

                break;
            case 3:
                m_commandPtr = std::make_unique<frc2::CommandPtr>(routine->Dynamic(frc2::sysid::Direction::kReverse));
                m_mode = 0;

                break;
            }

            m_stage = 1;
        }
    return;
    case 1: // Initialize.
        m_commandPtr->get()->Initialize();
        m_stage = 2;


    return;
    case 2: // Execute.
        if (m_commandPtr->get()->IsFinished())
        {
            m_stage = 3;
        }
        m_commandPtr->get()->Execute();

     return;
    case 3: // End.
        m_commandPtr->get()->End(false);
        m_stage = 0;

     ++m_routine;

        return;
    }
}

bool SysIdCommand::IsFinished() noexcept
{
    return m_routine >= 3;
}