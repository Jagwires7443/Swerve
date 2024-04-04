// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"
// #include "RobotContainer.h"

#include <frc/controller/ProfiledPIDController.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <units/angle.h>

void TimedAutoBase::Initialize() noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
    m_drive->ResetDrive();
    m_drive->ZeroHeading();

    IntakeSubsystem_->Default(0.0);
    IntakeSubsystem_->StopIntake();

    IntakeSubsystem_->StopTrigger();
    IntakeSubsystem_->StopOotas();

    finished_ = false;
    FPGATime_ = frc::RobotController::GetFPGATime();
    counter_ = 0;
}

void TimedAutoBase::End(bool interrupted) noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    IntakeSubsystem_->Default(0.0);
    IntakeSubsystem_->StopIntake();

    IntakeSubsystem_->StopTrigger();
    IntakeSubsystem_->StopOotas();
}

void TimedAutoBase::Execute() noexcept
{
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    const uint32_t deltaTime = (FPGATime - FPGATime_) / 1000; // ms

    if (deltaTime < 100) // 100ms
    {
        return;
    }

    if (Iteration(++counter_))
    {

        finished_ = true;
    }
}

bool BasicBack::Iteration(const uint32_t counter) noexcept
{
    if (counter <= 80)
    {
        m_drive->Drive(3_mps, 0_mps, 0_deg_per_s, false);
        return false;
    }
    if (counter <= 120) //
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
    }

    return true;
}

bool Shoot2Taxi::Iteration(const uint32_t counter) noexcept
{
    if (counter <= 40) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 80)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 120)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 160)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    if (counter <= 240)
    {
        IntakeSubsystem_->RunIntake();
        m_drive->Drive(1.5_mps, 0_mps, 0_deg_per_s, false);
        return false;
    }
    if (counter <= 310)
    {

        m_drive->Drive(-1.5_mps, 0_mps, 0_deg_per_s, false);
        arm_->ArmAtSpeaker();
        return false;
    }
    if (counter <= 360)
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        return false;
    }
    if (counter <= 380)
    {
        IntakeSubsystem_->FullOotas();
        return false;
    }
    if (counter <= 440) // 9 seconds
    {
        IntakeSubsystem_->RunTrigger();
        IntakeSubsystem_->StopIntake();
        return false;
    }
    return true;
}

bool test::Iteration(const uint32_t counter) noexcept
{
    if (counter <= 120) // 40 is 1 second
    {
        m_drive->Drive(1_mps, 0_mps, 0_deg_per_s, false);
        return false;
    }
    if (counter <= 160)
    {
        m_drive->Drive(0_mps, 0_mps, 90_deg_per_s, false);
        return false;
    }
    if (counter <= 200)
    {
        m_drive->SetTurnToAngle(90_deg);
        return false;
    }
    /*
        if (counter <= 120)
    {
        m_drive->SetTurnToAngle(135_deg);
        return false;
    }
        if (counter <= 160)
    {
        m_drive->Drive(1_mps, 0_mps, 0_deg_per_s, false);
        return false;
    }*/
    return true;
}

bool SideShoot1::Iteration(const uint32_t counter) noexcept
{
    if (counter <= 40) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 80)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 120)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 160)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    if (counter <= 240)
    {
        IntakeSubsystem_->RunIntake();
        m_drive->Drive(1.5_mps, 0_mps, 0_deg_per_s, false);
        return false;
    }
    if (counter <= 310)
    {
        m_drive->Drive(0.0_mps, 0_mps, 0_deg_per_s, false);
        IntakeSubsystem_->StopIntake();
    }
    return true;
}
// blue right side
bool OpenRightSide::Iteration(const uint32_t counter) noexcept // needs change
{
    if (counter <= 30) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 60)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 90)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 130)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    if (counter <= 190)
    {
        m_drive->Drive(3_mps, 0_mps, 92.0_deg_per_s, false); // make smaller needs to go 8ft around stage
        return false;
    }
    if (counter <= 195)
    {
        m_drive->Drive(0_mps, 0_mps, 0.0_deg_per_s, false);
        return false;
    }
    if (counter <= 260)
    {
        IntakeSubsystem_->RunIntake();
        m_drive->Drive(3_mps, 0_mps, 0.0_deg_per_s, false);
        return false;
    }
    if (counter <= 265)
    {
        IntakeSubsystem_->RunIntake();
        m_drive->Drive(0_mps, 0_mps, 0.0_deg_per_s, false);
        return false;
    }
    if (counter <= 320)
    {
        m_drive->Drive(-3.2_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 400)
    {
        m_drive->Drive(-3.2_mps, 0_mps, -78.0_deg_per_s, false);
        IntakeSubsystem_->StopIntake();
        return false;
    }
    if (counter <= 480)
    {
        m_drive->Drive(-3.2_mps, 0_mps, -0.0_deg_per_s, false); // could make from shootspeakerfar but less consistent
        return false;
    }
    if (counter <= 530)
    {
        m_drive->Drive(0_mps, 0_mps, -0.0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 580)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 640)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 780)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    return true;
}
// red left side 1 note
bool OpenLeftSide::Iteration(const uint32_t counter) noexcept
{
    if (counter <= 30) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 90)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 120)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 150)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    if (counter <= 210)
    {
        m_drive->Drive(3_mps, 0_mps, -62_deg_per_s, false);
        return false;
    }
    if (counter <= 310)
    {
        m_drive->Drive(3_mps, 0_mps, -32_deg_per_s, false);
        return false;
    }
    return true;
}

// right side is 3 note and from red side
bool RightSide::Iteration(const uint32_t counter) noexcept // test distance and shoot far
{
    if (counter <= 30) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 60)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 90)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 110)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    if (counter <= 140)
    {
        m_drive->Drive(3.5_mps, 0_mps, 300.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 220)
    {
        m_drive->Drive(2_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 280)
    {
        m_drive->Drive(-3.2_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 313)
    {
        m_drive->Drive(-5.5_mps, 0_mps, -330.0_deg_per_s, false);
        return false;
    }
    if (counter <= 319)
    {
        m_drive->Drive(-1_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->StopIntake();
        return false;
    }
    if (counter <= 329) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 355)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 375)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 395)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    if (counter <= 420)
    {
        m_drive->Drive(2.6_mps, 0_mps, 230.0_deg_per_s, false);
        return false;
    }
    if (counter <= 460)
    {
        IntakeSubsystem_->RunIntake();
        m_drive->Drive(5_mps, 0_mps, 0.0_deg_per_s, false);
        return false;
    }
    if (counter <= 560)
    {
        IntakeSubsystem_->RunIntake();
        m_drive->Drive(2_mps, 0_mps, 0.0_deg_per_s, false);
        return false;
    }
    if (counter <= 600)
    {
        m_drive->Drive(-5.5_mps, 0_mps, -18.0_deg_per_s, false);
        return false;
    }
    if (counter <= 840)
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->SpeakerFar();
        return false;
    }
    if (counter <= 870)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 900)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 930)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    return true;
}

bool LeftSide::Iteration(const uint32_t counter) noexcept
{
    if (counter <= 30) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 60)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 90)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 130)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    if (counter <= 190)
    {
        m_drive->Drive(3_mps, 0_mps, -92.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 195)
    {
        m_drive->Drive(0_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 230)
    {
        m_drive->Drive(2_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 235)
    {
        m_drive->Drive(0_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 275)
    {
        m_drive->Drive(-2_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 280)
    {
        m_drive->Drive(0_mps, 0_mps, 0.0_deg_per_s, false);
        IntakeSubsystem_->RunIntake();
        return false;
    }
    if (counter <= 400)
    {
        m_drive->Drive(-3_mps, 0_mps, 95.0_deg_per_s, false);
        IntakeSubsystem_->StopIntake();
        return false;
    }
    if (counter <= 430) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->Stow();
        return false;
    }
    if (counter <= 490)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 520)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 550)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    if (counter <= 610)
    {
        m_drive->Drive(3_mps, 0_mps, -81.0_deg_per_s, false);
        return false;
    }
    if (counter <= 615)
    {
        m_drive->Drive(0_mps, 0_mps, 0.0_deg_per_s, false);
        return false;
    }
    if (counter <= 690)
    {
        IntakeSubsystem_->RunIntake();
        m_drive->Drive(4_mps, 0_mps, 0.0_deg_per_s, false);
        return false;
    }
    if (counter <= 695)
    {
        m_drive->Drive(0_mps, 0_mps, 0.0_deg_per_s, false);
        return false;
    }
    if (counter <= 780)
    {
        m_drive->Drive(-3.5_mps, 0_mps, 30.0_deg_per_s, false);
        return false;
    }
    if (counter <= 870) // 40 is 1 second
    {
        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);
        arm_->SpeakerFar();
        return false;
    }
    if (counter <= 930)
    {
        IntakeSubsystem_->Ootas();
        return false;
    }
    if (counter <= 960)
    {
        IntakeSubsystem_->RunTrigger();
        return false;
    }
    if (counter <= 990)
    {
        IntakeSubsystem_->StopOotasandBodyT();
        arm_->ArmAtPickup();
        return false;
    }
    return true;
}

// trajectory
#if 0
frc2::CommandPtr TrajectoryAuto::TrajectoryAutoCommandFactory(DriveSubsystem *const drive, std::string_view name, frc::Trajectory &trajectory) noexcept
{
    frc2::SwerveControllerCommand<4> command
    {
trajectory, [drive]() -> frc::Pose2d
        { return drive->GetPose(); },
        drive->kDriveKinematics,
        frc2::PIDController{0.6, 0, 0},
        frc2::PIDController{0.6, 0, 0},
        frc::ProfiledPIDController<units::radians>{pidf::kDriveThetaP, pidf::kDriveThetaI, pidf::kDriveThetaD, frc::TrapezoidProfile<units::angle::radians>::Constraints{pidf::kDriveThetaMaxVelocity, pidf::kDriveThetaMaxAcceleration}},
        // [drive]() -> frc::Rotation2d "desiredRotation"
        [drive](std::array<frc::SwerveModuleState, 4> states) -> void
        { drive->SetModuleStates(states); },
        {drive}
    };

    command.SetName(name);

    return std::move(command).ToPtr();
}
#endif
