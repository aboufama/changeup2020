#include "vex.h"
#include "Util.h"
#include "Definitions.h"
#include "MovementFuncs.h"

double sp;

void TurnInPlace(float Deg, double slowDown, int minSpeed){
  

  float inertDif = Deg-inert;

  TR.spin(forward);// fires up motors
  TL.spin(forward);
  BR.spin(forward);
  BL.spin(forward);
   
   /* in the case of when a robot turns from 1 degree to 360
      it only really turns 1 degree, but it will read as -359 degrees, 
      that's why this loop is here. */

  while (inertDif*inertDif > 0.5){ // finds how far the robot is from the desired angle 
      inertDif =  Deg-inert; 
   
    if(inertDif > 180){
      inertDif -=360;
    }
     if(inertDif < -180){
      inertDif +=360;
    }
  
  if (inertDif>0){
  TR.setVelocity(((-inertDif)* abs((int) inertDif))/(slowDown * abs((int) inertDif))-minSpeed,pct);// fun pid stuff
  TL.setVelocity(((inertDif)* abs((int) inertDif))/(slowDown * abs((int) inertDif))+minSpeed,pct);
  BR.setVelocity(((-inertDif)* abs((int) inertDif))/(slowDown * abs((int) inertDif))-minSpeed,pct);
  BL.setVelocity(((inertDif)* abs((int) inertDif))/(slowDown * abs((int) inertDif))+minSpeed,pct);
  }else
  {
  TR.setVelocity(((-inertDif)* abs((int) inertDif))/(slowDown * abs((int) inertDif))+minSpeed,pct);// more fun pid stuff
  TL.setVelocity(((inertDif)* abs((int) inertDif))/(slowDown * abs((int) inertDif))-minSpeed,pct);
  BR.setVelocity(((-inertDif)* abs((int) inertDif))/(slowDown * abs((int) inertDif))+minSpeed,pct);
  BL.setVelocity(((inertDif)* abs((int) inertDif))/(slowDown * abs((int) inertDif))-minSpeed,pct);
  }


  vex::task::sleep(20); // sleep to not overload the system
  }


  StopMotorsEnd();// stops all motors
}



/*

Cartesian Strafe Function

*/


void StrafeXY(double x, double y, int speed, double decel){

double th = 90 - atan2(y,x)*(180/3.14159265);

double xst = xRot.position(degrees);
double yst = yRot.position(degrees);


TL.spin(forward);
TR.spin(forward);
BL.spin(forward);
BR.spin(forward);


if (th-inert<0){
  th = (3.14159265/180) * (450-(th-inert+360));
}else{

th = (3.14159265/180) * (450-(th-inert));
}


double px = -cos(th + 3.14159265/4);
double py = sin(th + 3.14159265/4);

double sp = (std::max(fabs(py),fabs(px)))/(speed*0.01);

double xDis = (xst+x)-xRot.position(degrees);
double yDis = (yst+y)-yRot.position(degrees);
double yVel = 1;
double xVel = 1;

while (fabs(xDis)>5 or fabs(yDis)>5){

xDis = (xst+x)-xRot.position(degrees);
yDis = (yst+y)-yRot.position(degrees);

yVel -= 0.01;
xVel -= 0.01;

TL.setVelocity(((py*yVel)/sp)*100,pct);
TR.setVelocity(((px*xVel)/sp)*100,pct);
BL.setVelocity(((px*xVel)/sp)*100,pct);
BR.setVelocity(((py*yVel)/sp)*100,pct);

vex::task::sleep(20);

}
StopMotorsEnd();
}



// different strafe function using polar instead of cartisean plane



void SetVelocityTheta(double th, double speed, std::string state){
TL.spin(forward);
TR.spin(forward);
BL.spin(forward);
BR.spin(forward);

if (state == "local"){
  th = (3.14159265/180) * (450-(th));
}else{
if (th-inert<0){
  th = (3.14159265/180) * (450-(th-inert+360));
}else{

th = (3.14159265/180) * (450-(th-inert));
}
}

double px = -cos(th + 3.14159265/4);
double py = sin(th + 3.14159265/4);

double sp = (std::max(fabs(py),fabs(px)))/(speed);

TL.setVelocity((py/sp)*100,pct);
TR.setVelocity((px/sp)*100,pct);
BL.setVelocity((px/sp)*100,pct);
BR.setVelocity((py/sp)*100,pct);


}