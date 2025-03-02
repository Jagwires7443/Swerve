#pragma once // This is a header guard. It prevents the header file from being included more than once.

#include "infrastructure/SparkMax.h"
#include <frc/motorcontrol/PWMVictorSPX.h> // Wat
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/button/CommandGenericHID.h>
#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h> 
#include <units/length.h>
#include "infrastructure/SparkMax.h" //motor controller?
#include <memory>
#include <string>
#include <frc/DigitalInput.h>
#include <frc/TimedRobot.h>

class ElevatorSubsystem : public frc::TimedRobot {
public:
    void RobotInit() override;
    void TeleopPeriodic() override;

private:
    frc::XboxController controller{0}; // Xbox controller on port 0
    frc::DigitalInput lowerLimitSwitch{0}; // Digital input channel 0
    frc::DigitalInput upperLimitSwitch{1}; // Digital input channel 1
    frc::PWMVictorSPX elevatorMotor{2}; // PWM channel 2

    void ControlElevator();
};