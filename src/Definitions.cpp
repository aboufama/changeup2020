#include "vex.h"
#include "Util.h"
#include "Definitions.h"

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       Definitions.cpp                                           */
/*    Author:       Andre Boufama                                             */
/*    Created:      Fri Jan 15 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/


void Init(){


StopMotorsEnd(); // stops mall the motors 
  
Inertial16.calibrate();
while(Inertial16.isCalibrating() ==true){ // calibrates inertial sensor
  vex::task::sleep(250);// waits while calibrating
}

xRot.resetPosition();//resets encoder values
yRot.resetPosition();

yTurnOffset = 234; /* how may degrees the Y encoder rotates 
in one 360 deg rotation of robot*/

xTurnOffset = 1603; // how may degrees the X encoder rotates in one 360 deg rotation of robot
inertError = 0.02365; // error of inertial sensor being used. Enter a number like 1.01 for 1% adjustment. 

xPos = 0; //Starting Positions
yPos = 0;

}


