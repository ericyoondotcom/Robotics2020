#include "vex.h"
#include <algorithm>

using namespace vex;

// Turn this on to make the robot able to go full speed when going diagonal. However, movement may feel less natural.
#define NORMALIZE_DIAGONALS true

brain Brain;
controller Controller;
motor MotorA = motor(PORT1, ratio18_1, false);
motor MotorB = motor(PORT11, ratio18_1, true);
motor MotorC = motor(PORT14, ratio18_1, true);
motor MotorD = motor(PORT10, ratio18_1, false);

void preDriver(){
  MotorA.setBrake(brakeType::hold);
  MotorB.setBrake(brakeType::hold);
  MotorC.setBrake(brakeType::hold);
  MotorD.setBrake(brakeType::hold);
}

int main() {
  preDriver();

  float speed = .5;

  while(true){

    if(Controller.ButtonX.pressing()){
      speed = 1;
    }
    else if(Controller.ButtonA.pressing()){
      speed = .5;
    }
    else if(Controller.ButtonB.pressing()){
      speed = .3;
    }

    float driveX = Controller.Axis4.position(percentUnits::pct);
    float driveY = Controller.Axis3.position(percentUnits::pct);
  #if NORMALIZE_DIAGONALS
    float denominator = std::max(std::abs(driveX), std::abs(driveY));
  #else
    float denominator = 100;
  #endif
    float normalizedX = denominator == 0 ? 0 : (driveX / denominator * 100);
    float normalizedY = denominator == 0 ? 0 : (driveY / denominator * 100);
    float rot = Controller.Axis1.position(percentUnits::pct);
    MotorA.spin(directionType::fwd, (normalizedX + normalizedY + rot) * speed, velocityUnits::pct);
    MotorB.spin(directionType::fwd, (normalizedX - normalizedY - rot) * speed, velocityUnits::pct);
    MotorC.spin(directionType::fwd, (normalizedX + normalizedY - rot) * speed, velocityUnits::pct);
    MotorD.spin(directionType::fwd, (normalizedX - normalizedY + rot) * speed, velocityUnits::pct);
    task::sleep(20);
  }
}
