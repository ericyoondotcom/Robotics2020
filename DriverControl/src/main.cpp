#include "vex.h"
#include <algorithm>
#include <cmath>
#include <stdlib.h>
using namespace vex;

#define CONTROLLER_DEADZONE 3

#define ROLLER_SPEED_FWD 185
#define ROLLER_SPEED_REV 160
#define INTAKE_SPEED_FWD 150
#define INTAKE_SPEED_REV 75

#define MACROS_ORTHOGONAL_SPEED 75

brain Brain;
controller Controller;

// REAL BRAIN
// motor MotorA = motor(PORT11, ratio18_1, false); // Front Left
// motor MotorB = motor(PORT3, ratio18_1, true); // Back Left
// motor MotorC = motor(PORT2, ratio18_1, true); // Back Right
// motor MotorD = motor(PORT1, ratio18_1, false); // Front Right
// motor IntakeL = motor(PORT14, ratio18_1, false);
// motor IntakeR = motor(/*PORT12*/ PORT16, ratio18_1, true);
// motor RollerF = motor(PORT13, ratio18_1, true);
// motor RollerB = motor(PORT15, ratio18_1, false);
// inertial Gyro = inertial(PORT12);

// 2ND BRAIN
motor MotorA = motor(PORT11, ratio18_1, false); // Front Left
motor MotorB = motor(PORT1, ratio18_1, true); // Back Left
motor MotorC = motor(PORT5, ratio18_1, true); // Back Right
motor MotorD = motor(PORT3, ratio18_1, false); // Front Right
motor IntakeL = motor(PORT15, ratio18_1, false);
motor IntakeR = motor(/*PORT12*/ PORT16, ratio18_1, true);
motor RollerF = motor(PORT16, ratio18_1, true);
motor RollerB = motor(PORT17, ratio18_1, false);
inertial Gyro = inertial(PORT13);

void preDriver(){
  MotorA.setBrake(brakeType::hold);
  MotorB.setBrake(brakeType::hold);
  MotorC.setBrake(brakeType::hold);
  MotorD.setBrake(brakeType::hold);
  IntakeL.setBrake(brakeType::brake);
  IntakeR.setBrake(brakeType::brake);
  RollerF.setBrake(brakeType::brake);
  RollerB.setBrake(brakeType::brake);
  Gyro.calibrate();
  while(Gyro.isCalibrating()){
    task::sleep(20);
  }
}

void pointTurn(float degrees){
  float degreesConversion = 6.5;
  float speed = 50;
  
  MotorA.rotateFor(degrees * degreesConversion, rotationUnits::deg, speed, velocityUnits::pct, false);
  MotorB.rotateFor(-degrees * degreesConversion, rotationUnits::deg, speed, velocityUnits::pct, false);
  MotorC.rotateFor(-degrees * degreesConversion, rotationUnits::deg, speed, velocityUnits::pct, false);
  MotorD.rotateFor(degrees * degreesConversion, rotationUnits::deg, speed, velocityUnits::pct, false);
  while(MotorA.isSpinning() || MotorB.isSpinning() || MotorC.isSpinning() || MotorD.isSpinning()){
    task::sleep(20);
  }
}

void turnToAngle(double targetHeading, float speedPct, bool useBestDirection = true, bool reverseForPrecision = true){
  const float TOLERANCE = 0;


  float currentHeading = Gyro.heading();

  Controller.Screen.setCursor(0, 0);
  Controller.Screen.clearScreen();
  Controller.Screen.newLine();
  Controller.Screen.print(currentHeading);

  bool condition = true;

  if(std::abs(targetHeading - currentHeading) < TOLERANCE) return;

  targetHeading = std::fmod(targetHeading, 360);

  if(useBestDirection){
    if(targetHeading > currentHeading){
      if(targetHeading - currentHeading > 180){
        speedPct = -std::abs(speedPct);
      }else{
        speedPct = std::abs(speedPct);
      }
    }else{
      if(currentHeading - targetHeading > 180){
        speedPct = std::abs(speedPct);
      }else{
        speedPct = -std::abs(speedPct);
      }
    }
  }


  if(targetHeading == 0) targetHeading = speedPct > 0 ? .1f : -.1f;

  MotorA.spin(directionType::fwd, speedPct, velocityUnits::pct);
  MotorB.spin(directionType::rev, speedPct, velocityUnits::pct);
  MotorC.spin(directionType::rev, speedPct, velocityUnits::pct);
  MotorD.spin(directionType::fwd, speedPct, velocityUnits::pct);

  while (condition){
    currentHeading = Gyro.heading();

    if(speedPct > 0){
      // turning right
      if(currentHeading > targetHeading){
        if(currentHeading - targetHeading > 180){
          // looping around from 360 to 0
          condition = true;
        }else{
          condition = false;
        }
      }else{
          if(targetHeading - currentHeading > 180){
          condition = false;
        }else{
          condition = true;
        }
      }
    }else{
      // turning left
      if(currentHeading < targetHeading){
        if(targetHeading - currentHeading > 180){
          // looping around from 360 to 0
          condition = true;
        }else{
          condition = false;
        }
      }else{
        if(currentHeading - targetHeading > 180){
          condition = false;
        }else{
          condition = true;
        }
      }
    }
  }

  if(reverseForPrecision){
    turnToAngle(targetHeading, speedPct * -0.3, false, false);
  }

  MotorA.stop();
  MotorB.stop();
  MotorC.stop();
  MotorD.stop();
}

int main() {
  preDriver();

  float speed = .5;

  while(true){
    double gyroReading = Gyro.heading();
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

    if(Controller.ButtonUp.pressing()){
      turnToAngle(0, MACROS_ORTHOGONAL_SPEED);
    }else if(Controller.ButtonRight.pressing()){
      turnToAngle(90, MACROS_ORTHOGONAL_SPEED);
    }else if(Controller.ButtonDown.pressing()){
      turnToAngle(180, MACROS_ORTHOGONAL_SPEED);
    }else if(Controller.ButtonLeft.pressing()){
      turnToAngle(270, MACROS_ORTHOGONAL_SPEED);
    }
  
    float driveX = Controller.Axis4.position();
    float driveY = Controller.Axis3.position();
    if(std::abs(driveX) < CONTROLLER_DEADZONE){
      driveX = 0;
    }
    if(std::abs(driveY) < CONTROLLER_DEADZONE){
      driveY = 0;
    }
    
    float euclidianDistance = std::sqrt(std::pow(driveX / 100, 2)  + std::pow(driveY / 100, 2));
    float stickAngle = driveY == 0 ? 0 : std::fmod((std::atan(driveY / driveX) * 180 / M_PI) + 360, 360);
    float relativeAngle = std::fmod(stickAngle + gyroReading, 360);

    if(euclidianDistance > 1) euclidianDistance = 1;

    float normalizedX = std::cos(relativeAngle * M_PI / 180) * euclidianDistance * 100 * (driveX < 0 ? -1 : 1); // Cos = adjacent (x) / hypotenuse (distance)
    float normalizedY = std::sin(relativeAngle * M_PI / 180) * euclidianDistance * 100 * (driveX < 0 ? -1 : 1); // Sin = opposite (y) / hypotenuse (distance)

    float rot = Controller.Axis1.position(percentUnits::pct);
    if(std::abs(rot) < CONTROLLER_DEADZONE){
      rot = 0;
    }
    MotorA.spin(directionType::fwd, (normalizedX + normalizedY + rot) * speed, velocityUnits::pct);
    MotorB.spin(directionType::fwd, (normalizedX - normalizedY - rot) * speed, velocityUnits::pct);
    MotorC.spin(directionType::fwd, (normalizedX + normalizedY - rot) * speed, velocityUnits::pct);
    MotorD.spin(directionType::fwd, (normalizedX - normalizedY + rot) * speed, velocityUnits::pct);

    if(Controller.ButtonY.pressing()){
      Controller.Screen.setCursor(0, 0);
      Controller.Screen.clearScreen();
      Controller.Screen.print(std::atan(driveY / driveX) * 180 / M_PI);
      Controller.Screen.newLine();
      Controller.Screen.print(stickAngle);
      Controller.Screen.newLine();
      Controller.Screen.print(rot);
      Controller.Screen.newLine();
    }

    if(Controller.ButtonR2.pressing()){
      bool reverse = Controller.ButtonL2.pressing();
    
      RollerF.spin(directionType::fwd, reverse ? ROLLER_SPEED_REV : ROLLER_SPEED_FWD, velocityUnits::rpm);
      RollerB.spin(reverse ? directionType::fwd : directionType::rev, reverse ? ROLLER_SPEED_REV : ROLLER_SPEED_FWD, velocityUnits::rpm);
    }else{
      RollerF.stop();
      RollerB.stop();
    }

    if(Controller.ButtonR1.pressing()){
      IntakeL.spin(directionType::fwd, INTAKE_SPEED_FWD, velocityUnits::rpm);
      IntakeR.spin(directionType::fwd, INTAKE_SPEED_FWD, velocityUnits::rpm);
    } else if(Controller.ButtonL1.pressing()){
      IntakeL.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::rpm);
      IntakeR.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::rpm);
    } else {
      IntakeL.stop();
      IntakeR.stop();
    }

    task::sleep(20);
  }
}

