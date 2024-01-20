# VEX Robotics Code Documentation: Autonomous and Teleoperated Control for VEX V5 Robot - Queen's Robotics Cup - 	[University of Toronto VEX Robotics Team (VEXUT)](https://www.robotevents.com/teams/VEXU/VEXUT1)

1. **Global Variables and Includes:**
   - The code starts with comments providing information about the module, author, creation date, and a brief description.
   - [It includes the necessary VEX header files and defines the motor and sensor configuration for the robot.](#vex-header-file)

2. **Structs:**
   - `ChassisDataSet`: A structure to hold data related to the chassis, such as positions of different motors, average position, difference, and heading from the inertial sensor.
   - `PIDDataSet`: A structure to store PID (Proportional, Integral, Derivative) constants.

3. **Zeroing Function:**
   - `Zeroing(bool dist, bool HDG)`: A function to reset motor positions and gyro heading. It takes two boolean parameters to determine whether to reset distances (motor positions) and heading (gyro).

4. **ChassisUpdate Function:**
   - `ChassisUpdate()`: Updates the ChassisDataSet structure with current motor positions and gyro heading.

5. **Brake and Coast Functions:**
   - `BStop()`: Sets all chassis motors to brake mode and stops them.
   - `CStop()`: Sets all chassis motors to coast mode and stops them.

6. **Wing Functions:**
   - `LWingB()`: Sets the left wing motor to hold mode and stops it.
   - `RWingB()`: Sets the right wing motor to hold mode and stops it.

7. **Move Function:**
   - `Move(int left, int right)`: Moves the robot based on specified left and right motor speeds.

8. **RunRoller Function:**
   - `RunRoller(int val)`: Runs the roller motor with a specified speed.

9. **PID Control Functions:**
   - `MoveEncoderPID`, `TurnMaxTimePID`, `TurnMaxTimePIDWOneSide`, `MoveTimePID`: Functions for implementing PID control for specific movements.

10. **Autonomous and User Control Functions:**
   - `autonomous()`: Placeholder for autonomous code.
   - `usercontrol()`: Main function for teleoperated control.
   - `DriveTask`, `ATask`, `PTask`: Concurrent tasks for driving, arm control, and puncher control, respectively.

11. **Main Function:**
   - `main()`: Sets up the competition callbacks, runs the pre-autonomous function, and enters a loop to prevent the main function from exiting.

This code structure is designed for a VEX robotics competition. It includes functions for motor control, sensor reading, and PID-based movement. The user control is handled in a loop, and there are separate tasks for driving, arm control, and puncher control. The code also includes placeholders for autonomous movements, which you can fill in with specific actions based on your robot's strategy.

## VEX header file

The provided VEXcode file includes the necessary declarations and configurations for VEX V5 robotics hardware.

1. **VEX Header and Namespace:**
   - `#include "vex.h"`: Includes the VEX header file, providing necessary definitions for VEX V5 hardware.
   - `using namespace vex;`: Utilizes the `vex` namespace, making VEX-specific classes and functions accessible without prefixing.

2. **Global Instances:**
   - `brain Brain;`: Declares a global instance of the `brain` class for interacting with the V5 Brain.
   - `controller Controller1 = controller(primary);`: Declares a global instance of the `controller` class for the primary controller.

3. **Motor Declarations:**
   - Motor declarations are made for various components of the robot, each specifying its port, gear ratio, and direction.
   - Examples:
      - `motor LUB = motor(PORT1, ratio18_1, false);`: Left Upper Back motor on port 1 with an 18:1 gear ratio.
      - `motor LUF = motor(PORT5, ratio18_1, true);`: Left Upper Front motor on port 5 with an 18:1 gear ratio (reversed direction).
      - Similar declarations for other motors like `RUB`, `RUF`, `LBB`, `LBF`, `LWing`, `RWing`, `Roller`, `RBB`, and `RBF`.

4. **Digital Outputs:**
   - `digital_out Wing = digital_out(Brain.ThreeWirePort.A);`: Declares a digital output for the wing control, using port A of the three-wire port on the Brain.
   - `digital_out Intake = digital_out(Brain.ThreeWirePort.H);`: Declares a digital output for the intake control, using port H of the three-wire port on the Brain.

5. **Inertial Sensor:**
   - `inertial Gyro = inertial(PORT3);`: Declares an inertial sensor on port 3 for measuring the robot's orientation (heading).

6. **Optical Sensor:**
   - `optical CataEyes = optical(PORT14);`: Declares an optical sensor on port 14, possibly for line following or detecting objects.

7. **Remote Control Code Enable/Disable:**
   - `bool RemoteControlCodeEnabled = true;`: Declares a boolean variable to enable or disable remote control code. This can be useful for toggling autonomous and user control modes.

8. **vexcodeInit Function:**
   - `void vexcodeInit( void ) { // nothing to initialize }`: A function called at the start of `int main()` for initializing code, tasks, and devices added using VEXcode Pro. It is currently empty.

This code serves as the initialization and configuration section of your VEXcode project. It defines the hardware components, their configurations, and global instances used in the main code.
