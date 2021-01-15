/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kellyzhang                                                */
/*    Created:      Sat Jan 09 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// TR                   motor         1               
// TL                   motor         2               
// BR                   motor         3               
// BL                   motor         4               
// RIntake              motor         5               
// LIntake              motor         6               
// yRot                 rotation      14              
// Inertial16           inertial      16              
// xRot                 rotation      15              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "Util.h"
#include "MovementFuncs.h"
#include "Definitions.h"
#include "Odom.h"
#include <cmath>

using namespace vex;

// Variable Definitions//

float inert;
double yTurnOffset; 
double xTurnOffset; 
double inertError; 
double xPos; 
double yPos;



int Async(){

while (true){
  BckGround(); // background operations; inertCorrection & odom
}

return(0);
}

void Threads(){
vex::thread t( Async );
}


/*---------------------------------------------\\
||                                             ||
||           KEEP  int main() CLEAN!!          ||
||                                             ||
\\---------------------------------------------*/
  
int main() {
  
  vexcodeInit();// Initializing Robot Configuration.
  Init(); // initializes Inertial sensor and variables
  Threads();// starts inertial correction thread
  // StrafeXY(360, 360, 100, 5);

  StopMotorsEnd();// turns off motors
  return 0; 

}
