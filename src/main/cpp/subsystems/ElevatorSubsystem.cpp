#include <frc/TimedRobot.h>
#include <frc/XboxController.h>
#include <frc/motorcontrol/MotorController.h>
#include <frc/motorcontrol/PWMVictorSPX.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DigitalInput.h>

#include "Constants.h"
#include <units/angle.h>
#include <units/torque.h>
#include <units/voltage.h>
#include <cmath>

class ElevatorSubsystem : public frc::TimedRobot {
public:
    void RobotInit() override {
        // Initialization code
    }

    void TeleopPeriodic() override {
        // Control the elevator during teleop period
        ControlElevator();
    }

private:
    frc::XboxController controller{0}; // Xbox controller on port 0
    frc::DigitalInput lowerLimitSwitch{0}; // Digital input channel 0
    frc::DigitalInput upperLimitSwitch{1}; // Digital input channel 1
    frc::PWMVictorSPX elevatorMotor{2}; // PWM channel 2

    void ControlElevator() {
        // Check Xbox controller inputs
        //this part will need to be changed (xbox controls are in different file)
        bool moveUp = controller.GetAButton();
        bool moveDown = controller.GetBButton();

        // Control the elevator motor based on limit switches and controller input
        if (moveUp && !upperLimitSwitch.Get()) {
            // Move the elevator up unless the upper limit switch is triggered
            elevatorMotor.Set(0.5);
        } else if (moveDown && !lowerLimitSwitch.Get()) {
            // Move the elevator down unless the lower limit switch is triggered
            elevatorMotor.Set(-0.5);
        } else {
            // Stop the motor if no buttons are pressed or limit switches are triggered
            elevatorMotor.Set(0.0);
        }

        // Display limit switch states on the SmartDashboard
        frc::SmartDashboard::PutBoolean("Lower Limit Switch", lowerLimitSwitch.Get());
        frc::SmartDashboard::PutBoolean("Upper Limit Switch", upperLimitSwitch.Get());
    }
};

#ifndef RUNNING_FRC_TESTS
int main() {
    return frc::StartRobot<ElevatorSubsystem>();
}
#endif