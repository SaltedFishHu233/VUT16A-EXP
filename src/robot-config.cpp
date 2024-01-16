#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor LUB = motor(PORT1, ratio18_1, false);
motor LUF = motor(PORT5, ratio18_1, true);
motor RUB = motor(PORT6, ratio18_1, true);
motor RUF = motor(PORT10, ratio18_1, false);
motor LBB = motor(PORT11, ratio18_1, true);
motor LBF = motor(PORT15, ratio18_1, false);
motor LWing = motor(PORT13, ratio36_1, true);
motor RWing = motor(PORT18, ratio36_1, false);
motor Roller = motor(PORT8, ratio18_1, false);
digital_out Wing = digital_out(Brain.ThreeWirePort.A);
digital_out Intake = digital_out(Brain.ThreeWirePort.H);
motor RBB = motor(PORT16, ratio18_1, false);
motor RBF = motor(PORT20, ratio18_1, true);
inertial Gyro = inertial(PORT3);
optical CataEyes = optical(PORT14);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}