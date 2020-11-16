#include "vex.h"
#include <algorithm>
#include <cmath>

using namespace vex;

// Turn this on to make the robot able to go full speed when going diagonal. However, movement may feel less natural.
#define NORMALIZE_DIAGONALS false

#define CONTROLLER_DEADZONE 3

brain Brain;
controller Controller;
motor MotorA = motor(PORT1, ratio18_1, false);
motor MotorB = motor(PORT11, ratio18_1, true);
motor MotorC = motor(PORT14, ratio18_1, true);
motor MotorD = motor(PORT8, ratio18_1, false);

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
      // speed = 0;
    }
  
    float driveX = Controller.Axis4.position();
    float driveY = Controller.Axis3.position();
    if(std::abs(driveX) < CONTROLLER_DEADZONE){
      driveX = 0;
    }
    if(std::abs(driveY) < CONTROLLER_DEADZONE){
      driveY = 0;
    }
    
  #if NORMALIZE_DIAGONALS
    float denominator = std::max(std::abs(driveX), std::abs(driveY));
    float euclidianDistance = std::sqrt(std::pow(driveX / 100, 2)  + std::pow(driveY / 100, 2));
    if(euclidianDistance > 1) euclidianDistance = 1;
  #else
    float denominator = 100;
    float euclidianDistance = 1;
  #endif
    if(Controller.ButtonDown.pressing()){
      Controller.Screen.setCursor(0, 0);
      Controller.Screen.clearScreen();
      // Controller.Screen.print(driveX);
      // Controller.Screen.newLine();
      // Controller.Screen.print(driveY);
      // Controller.Screen.newLine();
      // Controller.Screen.print(euclidianDistance);
      Controller.Screen.print(MotorD.temperature(temperatureUnits::fahrenheit));
      Controller.Screen.newLine();
      Controller.Screen.print(MotorA.temperature(temperatureUnits::fahrenheit));
      Controller.Screen.newLine();
      
    }
    float normalizedX = denominator == 0 ? 0 : (driveX / denominator * 100) * euclidianDistance;
    float normalizedY = denominator == 0 ? 0 : (driveY / denominator * 100) * euclidianDistance;
    float rot = Controller.Axis1.position(percentUnits::pct);
    if(std::abs(rot) < CONTROLLER_DEADZONE){
      rot = 0;
    }
    MotorA.spin(directionType::fwd, (normalizedX + normalizedY + rot) * speed, velocityUnits::pct);
    MotorB.spin(directionType::fwd, (normalizedX - normalizedY - rot) * speed, velocityUnits::pct);
    MotorC.spin(directionType::fwd, (normalizedX + normalizedY - rot) * speed, velocityUnits::pct);
    MotorD.spin(directionType::fwd, (normalizedX - normalizedY + rot) * speed, velocityUnits::pct);
    task::sleep(20);
  }
}
