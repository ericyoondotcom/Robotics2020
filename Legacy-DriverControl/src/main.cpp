#include "vex.h"
#include <algorithm>
#include <cmath>
#include <stdlib.h>
using namespace vex;

#define CONTROLLER_DEADZONE 3

#define ROLLER_SPEED_FWD 100
#define ROLLER_SPEED_REV 80
#define ROLLER_UNSTUCK_SPEED 50
#define INTAKE_SPEED_FWD 75
#define INTAKE_SPEED_REV 40

#define MACROS_ORTHOGONAL_SPEED 75

#define INTAKE_PULSE_TIME 150
#define CYCLE_TIME 20

brain Brain;
controller Controller;

motor MotorA = motor(PORT12, ratio18_1, false); // Front Left
motor MotorB = motor(PORT7, ratio18_1, true); // Back Left
motor MotorC = motor(PORT2, ratio18_1, true); // Back Right
motor MotorD = motor(PORT6, ratio18_1, false); // Front Right
motor IntakeL = motor(PORT11, ratio18_1, false);
motor IntakeR = motor(PORT1, ratio18_1, true);
motor RollerF = motor(PORT13, ratio18_1, true);
motor RollerB = motor(PORT10, ratio18_1, false);
inertial Gyro = inertial(PORT9);


float intakePulse = 0;

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

/**
 * Turns to face a specified heading, relative to the robot's starting position.
 * @param targetHeading       The angle, in degrees, to face.
 * @param speedPct            The speed, in percent units, to move at. Positive to turn right, negative to turn left.
 * @param useBestDirection    If set to true, the sign of speedPct will be disregarded and the shortest path (left or right) will be taken.
 * @param reverseForPrecision If set to true, robot will turn the opposite direction after turn is finished to compensate for overshooting the target angle.
 */
void turnToAngle(double targetHeading, float speedPct, bool useBestDirection = true, bool reverseForPrecision = true){
  const float TOLERANCE = 0;

  float currentHeading = Gyro.heading();

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

  
void onIntakePressed(){
  intakePulse = INTAKE_PULSE_TIME;
}

int main() {
  preDriver();
  Controller.ButtonR2.pressed(onIntakePressed);
  
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
      // Controller.Screen.setCursor(0, 0);
      // Controller.Screen.clearScreen();
      // Controller.Screen.print(std::atan(driveY / driveX) * 180 / M_PI);
      // Controller.Screen.newLine();

      // Gyro.setHeading(0, rotationUnits::deg);

      RollerF.spin(directionType::rev, ROLLER_UNSTUCK_SPEED, velocityUnits::pct);
      RollerB.spin(directionType::fwd, ROLLER_UNSTUCK_SPEED, velocityUnits::pct);
    } else if(Controller.ButtonR2.pressing()){
      bool reverse = Controller.ButtonL2.pressing();
    
      RollerF.spin(directionType::fwd, reverse ? ROLLER_SPEED_REV : ROLLER_SPEED_FWD, velocityUnits::pct);
      RollerB.spin(reverse ? directionType::fwd : directionType::rev, reverse ? ROLLER_SPEED_REV : ROLLER_SPEED_FWD, velocityUnits::pct);
    } else {
      RollerF.stop();
      RollerB.stop();
    }

    if(Controller.ButtonR1.pressing() || intakePulse > 0){
      IntakeL.spin(directionType::fwd, INTAKE_SPEED_FWD, velocityUnits::pct);
      IntakeR.spin(directionType::fwd, INTAKE_SPEED_FWD, velocityUnits::pct);
    }
    else if(Controller.ButtonL1.pressing()){
      IntakeL.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::pct);
      IntakeR.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::pct);
    } else {
      IntakeL.stop();
      IntakeR.stop();
    }

    if(intakePulse > 0){
      intakePulse -= CYCLE_TIME;
    }

    task::sleep(CYCLE_TIME);
  }
}

