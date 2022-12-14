# Swerve (for Scrappy the Robot)
C++ code for swerve drive: 2x SPARK MAX/NEO plus SRX MAG ENCODER per swerve module (i.e SDS Swerve Module); with Test Mode code.

See [SwerveSensorInterfaceBoard](https://github.com/Jagwires7443/SwerveSensorInterfaceBoard) for details on electrical connections.

Here are some of the features implemented in this code:

* Handling PWM input from SRX MAG ENCODER to derive absolute position;
* Handling SPARK MAX motor controllers;
* Profiled PID control of turning position;
* Using turning position to override commanded drive, when modules are not facing in commanded direction;
* Distance and velocity Profiled PID control of drive motors;
* Test Mode includes interactive adjustment of PID settings;
* Error handling for motor controllers, up to being able to test code on a roboRIO with no motor controllers;
* Test Mode uses Shuffleboard to create a tab for each swerve module, and a tab for the overall swerve drive;
* Step-by-step bring up procedure for swerve modules and drive system is documented in block comments (in [SwerveModule.h](https://github.com/Jagwires7443/Swerve/blob/master/src/main/include/subsystems/SwerveModule.h));
* Provides several Test Mode routines to automatically have robot drive various fixed test patterns;
* Logic to manage and save configuration of motor controller settings;
* Primitives useful for autonomous driving;
* Integrated with WPILib.

Please see (and/or use) [shuffleboard.json](https://github.com/Jagwires7443/Swerve/blob/master/shuffleboard.json) for suggested Shuffleboard settings.

![alt text](https://github.com/Jagwires7443/Swerve/blob/master/TestMode1.PNG?raw=true)

![alt text](https://github.com/Jagwires7443/Swerve/blob/master/TestMode2.PNG?raw=true)
