using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor LUB;
extern motor LUF;
extern motor RUB;
extern motor RUF;
extern motor LBB;
extern motor LBF;
extern motor Roller;
extern digital_out Wing;
extern digital_out Intake;
extern motor RBB;
extern motor RBF;
extern inertial Gyro;
extern optical CataEyes;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );