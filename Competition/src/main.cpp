#include "vex.h"
#include <algorithm>
#include <cmath>
#include <stdlib.h>
using namespace vex;

// ************
#define SKILLS true
#define RED_TEAM true
#define RIGHT_SIDE_AUTON true
// ************

#define CONTROLLER_DEADZONE 3

#define ROLLER_SPEED_FWD 100
#define ROLLER_SPEED_REV 80
#define ROLLER_UNSTUCK_SPEED 50
#define INTAKE_SPEED_FWD 75
#define INTAKE_SPEED_REV 40

#define MACROS_ORTHOGONAL_SPEED 75

#define INTAKE_PULSE_TIME 150
#define CYCLE_TIME 20

#define MOTOR_TIMEOUT_SECS 5


competition Competition;
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

enum cardinal {
  forward,
  reverse,
  left,
  right
};

void setupRobot(){
  MotorA.setBrake(brakeType::hold);
  MotorB.setBrake(brakeType::hold);
  MotorC.setBrake(brakeType::hold);
  MotorD.setBrake(brakeType::hold);
  IntakeL.setBrake(brakeType::brake);
  IntakeR.setBrake(brakeType::brake);
  RollerF.setBrake(brakeType::brake);
  RollerB.setBrake(brakeType::brake);

  MotorA.setTimeout(MOTOR_TIMEOUT_SECS, timeUnits::sec);
  MotorB.setTimeout(MOTOR_TIMEOUT_SECS, timeUnits::sec);
  MotorC.setTimeout(MOTOR_TIMEOUT_SECS, timeUnits::sec);
  MotorD.setTimeout(MOTOR_TIMEOUT_SECS, timeUnits::sec);
  IntakeL.setTimeout(MOTOR_TIMEOUT_SECS, timeUnits::sec);
  IntakeR.setTimeout(MOTOR_TIMEOUT_SECS, timeUnits::sec);
  RollerF.setTimeout(MOTOR_TIMEOUT_SECS, timeUnits::sec);
  RollerB.setTimeout(MOTOR_TIMEOUT_SECS, timeUnits::sec);
  Gyro.calibrate();
  while(Gyro.isCalibrating()){
    wait(20, msec);
  }
  // Controller.Screen.setCursor(0, 0);
  // Controller.Screen.clearScreen();
  // Controller.Screen.print("CALIBRATED!");
  // Controller.Screen.newLine();
}

void moveCardinal(cardinal direction, float inches, float speed = 35, float timeout = 10000){
  float inchToRev = 1.0 / 11; // Theoretically this should be the circumference of the wheel, but x drive has lots of slipping 

  switch(direction){
    case cardinal::forward: {
      MotorA.rotateFor(inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorB.rotateFor(-inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorC.rotateFor(inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorD.rotateFor(-inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      break;
    }
    case cardinal::reverse: {
      MotorA.rotateFor(-inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorB.rotateFor(inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorC.rotateFor(-inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorD.rotateFor(inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      break;
    }
    case cardinal::left: {
      MotorA.rotateFor(-inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorB.rotateFor(-inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorC.rotateFor(-inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorD.rotateFor(-inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      break;
    }
    case cardinal::right: {
      MotorA.rotateFor(inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorB.rotateFor(inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorC.rotateFor(inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      MotorD.rotateFor(inches * inchToRev, rotationUnits::rev, speed, velocityUnits::pct, false);
      break;
    }
  }
  float t = 0;
  while(MotorA.isSpinning() || MotorB.isSpinning() || MotorC.isSpinning() || MotorD.isSpinning()){
    task::sleep(20);
    t += 20;
    if(t > timeout) break;
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

void spinIntakes(directionType direction){
  if(direction == directionType::fwd){
    IntakeL.spin(directionType::fwd, INTAKE_SPEED_FWD, velocityUnits::pct);
    IntakeR.spin(directionType::fwd, INTAKE_SPEED_FWD, velocityUnits::pct);
  }else if(direction == directionType::rev){
    IntakeL.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::pct);
    IntakeR.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::pct);
  }
}

void stopIntakes(){
  IntakeL.stop();
  IntakeR.stop();
}

void spinRollers(directionType direction, float speed = 0){
  if(direction == directionType::fwd){
    RollerF.spin(directionType::fwd, speed == 0 ? ROLLER_SPEED_FWD : speed, velocityUnits::pct);
    RollerB.spin(directionType::rev, speed == 0 ? ROLLER_SPEED_FWD : speed, velocityUnits::pct);
  }else if(direction == directionType::rev){
    RollerF.spin(directionType::fwd, speed == 0 ? ROLLER_SPEED_REV : speed, velocityUnits::pct);
    RollerB.spin(directionType::fwd, speed == 0 ? ROLLER_SPEED_REV : speed, velocityUnits::pct);
  }
}

void stopRollers(){
  RollerF.stop();
  RollerB.stop();
}
  
void onIntakePressed(){
  intakePulse = INTAKE_PULSE_TIME;
}


  void preDriver(){
    if(SKILLS){
      setupRobot();
    }
  }

void usercontrol(void) {
  while(Gyro.isCalibrating()){
    wait(20, msec);
  }
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


void pre_auton(void) {
  vexcodeInit();
  setupRobot();
}


void matchAutonomous(void){
  while(Gyro.isCalibrating()){
    wait(20, msec);
  }
}

void skillsAutonomous(void) {
  while(Gyro.isCalibrating()){
    wait(20, msec);
  }
  // AT START:
  // The robot should be aligned so it is centered with the ball on the field towards the left.
  // The preload should be pushed up so it's touching the two back rollers and the bottom front roller.

  // Move from wall to tower and rotate
  spinIntakes(fwd);
  moveCardinal(cardinal::forward, 12);
  task::sleep(300);
  stopIntakes();
  turnToAngle(270, 75);
  moveCardinal(cardinal::forward, 16);
  turnToAngle(180 + 45, 50);
  spinIntakes(fwd);
  moveCardinal(cardinal::forward, 8);
  // robot is on Red Left Tower
  spinRollers(fwd);
  spinIntakes(fwd);
  task::sleep(650);
  stopRollers();
  stopIntakes();
  RollerF.spin(directionType::rev, ROLLER_UNSTUCK_SPEED, velocityUnits::pct);
  task::sleep(500);
  stopRollers();
  spinIntakes(fwd);
  moveCardinal(cardinal::reverse, 10);
  spinIntakes(directionType::rev);
  moveCardinal(cardinal::reverse, 7);
  turnToAngle(180, 75);
  // Move the red ball up, and release a blue ball
  // spinRollers(directionType::fwd);
  // task::sleep(400);
  // spinRollers(directionType::rev);
  // task::sleep(600);
  stopRollers();
  moveCardinal(cardinal::left, 42);
  // robot is on Red Center Tower
  moveCardinal(cardinal::forward, 12.5, 35, 1000);
  spinRollers(fwd);
  spinIntakes(fwd);
  task::sleep(1200);
  stopRollers();
  moveCardinal(cardinal::reverse, 10);
  // Now off the center tower
  turnToAngle(90, 75);
  spinRollers(directionType::rev);
  task::sleep(1000);
  spinIntakes(directionType::rev);
  moveCardinal(cardinal::forward, 28);
  spinIntakes(directionType::fwd);
  stopRollers();
  // at this point the robot is picking up a field ball
  moveCardinal(cardinal::forward, 18);
  stopIntakes();
  stopRollers();
  turnToAngle(90 + 45, 47);
  spinIntakes(fwd);
  moveCardinal(cardinal::forward, 22.5, 35, 2000);
  // robot is on Red Right Tower
  spinRollers(fwd);
  task::sleep(1100);
  stopRollers();
  moveCardinal(cardinal::reverse, 15);
}

int main() {
  if(SKILLS){
    Competition.autonomous(skillsAutonomous);
  }else{
    Competition.autonomous(matchAutonomous);
  }
  Competition.drivercontrol(usercontrol);

  pre_auton();

  while (true) {
    wait(100, msec);
  }
}
