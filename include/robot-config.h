using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor TR;
extern motor TL;
extern motor BR;
extern motor BL;
extern motor RIntake;
extern motor LIntake;
extern rotation yRot;
extern inertial Inertial16;
extern rotation xRot;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );