#include "vex.h"
#include <algorithm>
#include <cmath>
#include <stdlib.h>
#include <iostream>
using namespace vex;

// ************
#define DEBUG false
#define PRINT_TEMPERATURES false
#define AUTON_NOT_PRELOADED false // Set to true if you're too lazy to tie back the arms for auton
#define SKILLS false
#define LIVE_REMOTE true
#define RED_TEAM true
// ************

#define ENCODER_L_DIST 3.75
#define ENCODER_R_DIST 3.75
#define ENCODER_B_DIST 3.75
#define ODOM_WHEEL_RADIUS 2.75 / 2

#define CONTROLLER_DEADZONE 3

#define ROLLER_SPEED_FWD 100
#define ROLLER_SPEED_REV 80
#define ROLLER_UNSTUCK_SPEED 80
#define INTAKE_SPEED_FWD 100
#define INTAKE_SPEED_FWD_AUTON 80
#define INTAKE_SPEED_REV 21
#define INTAKE_OPEN_POS 75
#define AUTON_INTAKE_OPEN_POSITION 50

#define MACROS_ORTHOGONAL_SPEED 75

// #define INTAKE_PULSE_TIME 150
#define INTAKE_PULSE_TIME 0
#define CYCLE_TIME 20

#define MOTOR_TIMEOUT_SECS 5

#define BUTTON_COOLDOWN_MILLIS 400

#define ROTATION_SOURCE rotationSource::inertial

competition Competition;
brain Brain;
controller Controller;

motor MotorA = motor(PORT13, ratio18_1, false); // Front Left
motor MotorB = motor(PORT7, ratio18_1, true); // Back Left
motor MotorC = motor(PORT2, ratio18_1, true); // Back Right
motor MotorD = motor(PORT6, ratio18_1, false); // Front Right
motor IntakeL = motor(PORT11, ratio18_1, false);
motor IntakeR = motor(PORT1, ratio18_1, true);
motor RollerF = motor(PORT14, ratio18_1, true);
motor RollerB = motor(PORT10, ratio18_1, false);
inertial Gyro = inertial(PORT9);
encoder EncoderR = encoder(Brain.ThreeWirePort.C);
encoder EncoderL = encoder(Brain.ThreeWirePort.G);
encoder EncoderB = encoder(Brain.ThreeWirePort.E);
bumper BumperL = bumper(Brain.ThreeWirePort.A);
bumper BumperR = bumper(Brain.ThreeWirePort.B);

bool enableRelativeDriving = true;

float encLPrev = 0;
float encRPrev = 0;
float encBPrev = 0;
#if SKILLS
float posX = 35.3f; // X is left/right
float posY = 0.0f; // Y is forwards/backwards
#define AUTON_STARTING_ROTATION 0
#else
float posX = 67.0f;
float posY = 7.0f;
#define AUTON_STARTING_ROTATION 297
#endif
float currRot = 0.0f;

float intakePulse = 0;

// Cooldowns:
// Is in the range [BUTTON_COOLDOWN_MILLIS, -1]
// -1 is idle; nothing will happen
// 0 is when it triggers. When a button is pressed it gets reset to BUTTON_COOLDOWN_MILLIS

float directionalButtonCooldown = -1;
float bButtonCooldown = -1;

bool upBtnFlag = false;
bool rightBtnFlag = false;
bool downBtnFlag = false;
bool leftBtnFlag = false;
bool bBtnFlag = false;
bool yBtnFlag = false;

enum cardinal {
  forward,
  reverse,
  left,
  right
};

enum rotationSource {
  odometry,
  inertial,
  average
};

void setupRobot(){
  MotorA.setBrake(brakeType::coast);
  MotorB.setBrake(brakeType::coast);
  MotorC.setBrake(brakeType::coast);
  MotorD.setBrake(brakeType::coast);
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

  EncoderL.resetRotation();
  EncoderR.resetRotation();
  EncoderB.resetRotation();
  Gyro.calibrate();
  while(Gyro.isCalibrating()){
    vex::this_thread::sleep_for(20);
  }
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
    vex::this_thread::sleep_for(20);
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
    vex::this_thread::sleep_for(20);
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

  float currentHeading;
  if(ROTATION_SOURCE == rotationSource::odometry){
    currentHeading = currRot * 180 / M_PI;
  } else if(ROTATION_SOURCE == rotationSource::inertial){
    currentHeading = Gyro.heading();
  } else if(ROTATION_SOURCE == rotationSource::average){
    currentHeading = ((currRot * 180 / M_PI) + Gyro.heading()) / 2;
  }

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
    vex::this_thread::sleep_for(20);
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
    IntakeL.spin(directionType::fwd, INTAKE_SPEED_FWD_AUTON, velocityUnits::pct);
    IntakeR.spin(directionType::fwd, INTAKE_SPEED_FWD_AUTON, velocityUnits::pct);
  }else if(direction == directionType::rev){
    IntakeL.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::pct);
    IntakeR.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::pct);
  }
}

void stopIntakes(brakeType brake = brakeType::coast){
  IntakeL.stop(brake);
  IntakeR.stop(brake);
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

directionType _ra_direction;
float _ra_revolutions;
float _ra_speed;
float _ra_timeout;
int spinRollersForCallback(){
  float t = 0;
  if(_ra_direction == directionType::fwd){
    RollerF.startRotateFor(directionType::fwd, _ra_revolutions, rotationUnits::rev, _ra_speed, velocityUnits::pct);
    RollerB.startRotateFor(directionType::rev, _ra_revolutions, rotationUnits::rev, _ra_speed, velocityUnits::pct);
  }else if(_ra_direction == directionType::rev){
    RollerF.startRotateFor(directionType::fwd, _ra_revolutions, rotationUnits::rev, _ra_speed, velocityUnits::pct);
    RollerB.startRotateFor(directionType::fwd, _ra_revolutions, rotationUnits::rev, _ra_speed, velocityUnits::pct);
  }
  while(t < _ra_timeout && !RollerF.isDone() && !RollerB.isDone()){
    t += CYCLE_TIME;
    vex::this_thread::sleep_for(CYCLE_TIME);
  }
  return 0;
}

thread spinRollersForAsync(directionType direction, float revolutions, float speed = 0, float timeout = 3000){
  _ra_direction = direction;
  _ra_revolutions = revolutions;
  if(speed == 0){
    _ra_speed = direction == directionType::fwd ? ROLLER_SPEED_FWD : ROLLER_SPEED_REV;
  }else{
    _ra_speed = speed;
  }
  _ra_timeout = timeout;
  return thread(spinRollersForCallback);
}
  
void onIntakePressed(){
  intakePulse = INTAKE_PULSE_TIME;
}


void preDriver(){
  if(SKILLS){
    setupRobot();
  }
}


int odometryTaskCallback(){
#if PRINT_TEMPERATURES
  float tempTime = 0;
#endif
  while(true){

#if !DEBUG
  if(!Competition.isAutonomous()){
    vex::this_thread::sleep_for(CYCLE_TIME);
    continue;
  }
#endif

#if PRINT_TEMPERATURES
  if(tempTime > 2000){
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(0, 0);
    Controller.Screen.print("L: ");
    Controller.Screen.print(IntakeL.temperature());
    Controller.Screen.newLine();
    Controller.Screen.print("R: ");
    Controller.Screen.print(IntakeR.temperature());
    tempTime = 0;
  }
  tempTime += 10;
#endif

    // Odometry algorithm was super helpfully explained by Team 5225 PiLons. Thanks!

    float encLNew = -EncoderL.rotation(rotationUnits::rev) * M_PI * 2;
    float encRNew = EncoderR.rotation(rotationUnits::rev) * M_PI * 2;
    float encBNew = EncoderB.rotation(rotationUnits::rev) * M_PI * 2;
    
    float deltaL = encLNew - encLPrev;
    float deltaR = encRNew - encRPrev;
    float deltaB = encBNew - encBPrev;
    encLPrev = encLNew;
    encRPrev = encRNew;
    encBPrev = encBNew;

    // Get the arc lengths travelled by the wheels
    // Just multiply by the wheel radius due to radians pog!
    float arcL = deltaL * ODOM_WHEEL_RADIUS;
    float arcR = deltaR * ODOM_WHEEL_RADIUS;
    float arcB = deltaB * ODOM_WHEEL_RADIUS;

    // Calculate the angle of the robot's arc path, therefore the rotation of the bot
    float deltaRot = (arcL - arcR) / (ENCODER_L_DIST + ENCODER_R_DIST);
    currRot += deltaRot;
    // Make positive and wrap around 2pi 
    currRot = std::fmod(currRot, 2.0f * M_PI);
    if(currRot < 0) {
      currRot += (2.0f * M_PI);
    }

    // Net movement (cartesian coords) since last cycle
    float deltaX;
    float deltaY;

    if(deltaRot == 0){
      // Special case where there was no rotation change
      deltaX = arcB;
      deltaY = (arcL + arcR) / 2; // get the avg just for a bit more precision
    } else {
      float arcRadiusX = (arcB / deltaRot) + ENCODER_B_DIST; // The radius of the robot's secondary arc path
      float arcRadiusY = (arcR / deltaRot) + ENCODER_R_DIST; // The radius of the robot's arc path. // TODO: Consider doing this for both L and R then taking avg?

      // Calculate the chords of the arc path, which will be the delta x and y in coordinate space
      // Chord length formula: 2 * r * sin(theta / 2)
      deltaX = 2 * std::sin(deltaRot / 2.0f) * arcRadiusX;
      deltaY = 2 * std::sin(deltaRot / 2.0f) * arcRadiusY;
    }

    // Delta movement needs to be rotated to be relative to the field. This should be the same algo as relative driving!
    float euclidianDistance = sqrt(std::pow(deltaX, 2) + std::pow(deltaY, 2));
    float vectorAngle = 0;
    if(deltaX == 0){
      if(deltaY < 0) vectorAngle = 0;
      else vectorAngle = M_PI;
    } else {
      vectorAngle = std::fmod(
        // Add 2pi to make it positive; then, another pi/2 to make UP zero, rather than RIGHT zero
        std::atan(deltaY / deltaX) + (2.5f * M_PI),
        (2.0f * M_PI)
      );
    }

    float currentHeading;
    if(ROTATION_SOURCE == rotationSource::odometry){
      currentHeading = currRot;
    } else if(ROTATION_SOURCE == rotationSource::inertial){
      currentHeading = Gyro.heading() * M_PI / 180;
    } else if(ROTATION_SOURCE == rotationSource::average){
      currentHeading = ((Gyro.heading() * M_PI / 180) + currRot) / 2;
    }

    float relativeAngle = std::fmod((double)(currentHeading + vectorAngle), (double)(2.0f * M_PI));

    float normalizedY = std::cos(relativeAngle) * euclidianDistance * (deltaX < 0 ? -1.0f : 1.0f);
    float normalizedX = std::sin(relativeAngle) * euclidianDistance * (deltaX < 0 ? -1.0f : 1.0f);
    
    posY += -normalizedY;
    posX += -normalizedX;

#if DEBUG
    if(Controller.ButtonY.pressing()){
      std::cout << "[\t" << posX << ",\t\t" << posY << "\t] \t @ \t odom: " << (currRot / (2 * M_PI) * 360) << "°\t\tgyro: " << Gyro.heading() << "°\n";
      // std::cout << -EncoderL.rotation(rotationUnits::rev) << ",\t\t" << EncoderR.rotation(rotationUnits::rev) << ",\t\t" << EncoderB.rotation(rotationUnits::rev) << std::endl;
    }
#endif
    vex::this_thread::sleep_for(CYCLE_TIME);
  }
  return 0;
}

float MAX_ERROR_XY = 2 * 24 * sqrt(2); // Max distance you can be off = hypotenuse of field
const float XY_KP = 7;
const float XY_KD = 2;
const float ROT_KP = 2.0f; // Remember: number must scale up to 100, but its in degrees
const float ROT_KD = 0;
float ERROR_THRESHOLD_XY = 1 * sqrt(2); // 1 inches in both directions allowed
float ERROR_THRESHOLD_ROT = 1; // Rotation error is in degrees

#if SKILLS
  const float MIN_XY_SPEED = 5;
  const float MAX_XY_SPEED = 80;
  const float MIN_ROT_SPEED = 10;
  const float MAX_ROT_SPEED = 50;
#else
  const float MIN_XY_SPEED = 5;
  const float MAX_XY_SPEED = 80;
  const float MIN_ROT_SPEED = 10;
  const float MAX_ROT_SPEED = 50;
#endif

int smartmove(float x, float y, float rotDeg, float timeout = 5000, bool stopOnBumperPress = false, bool doRotation = true, float minXYSpeed = MIN_XY_SPEED, float maxXYSpeed = MAX_XY_SPEED, float minRotSpeed = MIN_ROT_SPEED, float maxRotSpeed = MAX_ROT_SPEED, float errorThresholdXY = ERROR_THRESHOLD_XY, float errorThresholdRot = ERROR_THRESHOLD_ROT){
  float errorXY = infinityf();
  float errorRot = infinityf();
  float rot = rotDeg * M_PI / 180;
  bool hasCompletedTranslation = false;
  bool hasCompletedRotation = !doRotation;

  float t = 0;
  while(!hasCompletedTranslation || !hasCompletedRotation){
    if((BumperL.pressing() || BumperR.pressing()) && stopOnBumperPress){
      break;
    }

    float xDiff = posX - x;
    float yDiff = y - posY;
    float oldErrorXY = errorXY;
    float oldErrorRot = errorRot;
    float _currRot;
    if(ROTATION_SOURCE == rotationSource::odometry){
      _currRot = currRot;
    } else if(ROTATION_SOURCE == rotationSource::inertial){
      _currRot = Gyro.heading() * M_PI / 180;
    } else if(ROTATION_SOURCE == rotationSource::average){
      _currRot = ( (Gyro.heading() * M_PI / 180) + currRot ) / 2;
    }
    bool turnRight;
    // determine best direction
    if(rot > _currRot){
      turnRight = (rot - _currRot <= M_PI);
    }else{
      turnRight = (_currRot - rot > M_PI);
    }

    errorXY = sqrt(pow(yDiff, 2) + pow(xDiff, 2));
    errorRot = std::fmod ( std::abs(_currRot - rot) * (180 / M_PI), 360);
    if(errorRot > 180){
      errorRot = 360 - errorRot;
    }
    
    if(errorXY < errorThresholdXY && !hasCompletedTranslation){
      hasCompletedTranslation = true;
    }

    // Dont allow rotation to stop until translation has completed
    if(errorRot < errorThresholdRot && !hasCompletedRotation && hasCompletedTranslation){
      hasCompletedRotation = true;
#if DEBUG
      Controller.rumble(".");
      std::cout << "[\t" << posX << ",\t\t" << posY << "\t] \t @ \t " << (currRot / (2 * M_PI) * 360) << "°\n";
#endif
    }

    float speedXY;
    if(hasCompletedTranslation){
      speedXY = 0;
    } else {
      speedXY = (XY_KP * errorXY) + (XY_KD * std::abs(errorXY - oldErrorXY));
      if(speedXY > maxXYSpeed) speedXY = maxXYSpeed;
      else if(speedXY < minXYSpeed) speedXY = minXYSpeed;
    }
    
    float rotSpeed;
    if(hasCompletedRotation){
      rotSpeed = 0;
    } else {
      rotSpeed = (ROT_KP * errorRot) + (ROT_KD * std::abs(errorRot - oldErrorRot));
      if(rotSpeed > maxRotSpeed) rotSpeed = maxRotSpeed;
      else if(rotSpeed < minRotSpeed) rotSpeed = minRotSpeed;
    }

    float theta = std::fmod(std::atan(xDiff / yDiff) + (2 * M_PI), 2 * M_PI);
    theta = std::fmod(theta + _currRot + (M_PI / 2), 2 * M_PI);
    float normalizedX = std::cos(theta) * speedXY * (yDiff < 0 ? -1 : 1);
    float normalizedY = std::sin(theta) * speedXY * (yDiff < 0 ? -1 : 1);
    
    rotSpeed *= turnRight ? 1 : -1;

    MotorA.spin(directionType::fwd, normalizedX + normalizedY + rotSpeed, velocityUnits::pct);
    MotorB.spin(directionType::fwd, normalizedX - normalizedY - rotSpeed, velocityUnits::pct);
    MotorC.spin(directionType::fwd, normalizedX + normalizedY - rotSpeed, velocityUnits::pct);
    MotorD.spin(directionType::fwd, normalizedX - normalizedY + rotSpeed, velocityUnits::pct);

#if DEBUG
    Controller.Screen.clearScreen();
    Controller.Screen.setCursor(0, 0);
    Controller.Screen.print(theta * 180 / M_PI);
    if(Controller.ButtonY.pressing()){
      std::cout <<
        "[" << posX << ",\t" << posY << "]\t\t XYdiff: [" << xDiff << ",\t" << yDiff << "]\t\t\terrorXY: " << errorXY <<
        (turnRight ? "\t\t\t>>>" : "\t\t\t<<<") << "\t\ttheta " << (theta * 180 / M_PI) << "\t\tcurrRot: " << (_currRot * 180 / M_PI) << "\t\trotTarget: " << (rot * 180 / M_PI) << "\t\t rotError: " << errorRot <<
        "\t\t\tSpeedXY: " << speedXY << "\t\tSpeedRot: " << rotSpeed <<
        (hasCompletedTranslation ? "\t\tT" : "\t\t.") << (hasCompletedRotation ? "\t\tR" : "\t\t.") << std::endl;
      // std::cout << "errXY: " << errorXY << "\t\terrRot: " << errorRot << "\t\tSpeedXY: " << speedXY << "\t\tSpeedRot: " << rotSpeed << std::endl;
      // std::cout << "errRot: " << errorRot << "\t\tSpeedXY: " << speedXY << "\t\tSpeedRot: " << rotSpeed << std::endl;
      // std::cout << "OG Theta: " << std::fmod(std::atan(yDiff / xDiff) + (2 * M_PI), 2 * M_PI) * 180 / M_PI << "\t\tcurrRot: " << _currRot * 180 / M_PI << "\t\tTheta: " << (theta * 180 / M_PI) << std::endl;
    }
#endif

    t += 10;
    vex::this_thread::sleep_for(10);

    if(t >= timeout){
      break;
    }
  }
  MotorA.stop();
  MotorB.stop();
  MotorC.stop();
  MotorD.stop();
  return 0;
}

void usercontrol(void) {
  float t = 0;
  bool timeNotifSounded = false;
  while(Gyro.isCalibrating()){
    t += 20;
    vex::this_thread::sleep_for(20);
  }

#if DEBUG && !SKILLS
  Gyro.setHeading(AUTON_STARTING_ROTATION, rotationUnits::deg);
  currRot = (AUTON_STARTING_ROTATION / 180.0f) * M_PI;
  std::cout << Gyro.heading() << std::endl;
#endif

  MotorA.setBrake(brakeType::coast);
  MotorB.setBrake(brakeType::coast);
  MotorC.setBrake(brakeType::coast);
  MotorD.setBrake(brakeType::coast);

  IntakeL.resetRotation();
  IntakeR.resetRotation();
  Controller.ButtonR2.pressed(onIntakePressed);
  
  float speed = 1;
  bool holdingIntakes = false;
  bool holdingIntakes180 = false;
 
  while(true){
    double gyroReading = Gyro.heading();
    if(Controller.ButtonX.pressing()){
      speed = 1;
    }
    else if(Controller.ButtonA.pressing()){
      speed = .5;
    }
    else if(Controller.ButtonB.pressing()){
      // speed = .3;
      // We don't need lowest speed in Change Up, so use for emergency actions instead
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
    float stickAngle = driveY == 0 ? 0 : std::fmod((std::atan(driveY / driveX) * 180 / M_PI) + 360, 360); // Why is the ternary condition driveY, not driveX?
    float relativeAngle = enableRelativeDriving ? std::fmod(stickAngle + gyroReading, 360) : stickAngle;

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

#if !DEBUG
    if(Controller.ButtonY.pressing()){
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
#endif

    if(Controller.ButtonR1.pressing() || intakePulse > 0){
      IntakeL.setBrake(brakeType::coast);
      IntakeR.setBrake(brakeType::coast);
      holdingIntakes = false;
      holdingIntakes180 = false;
      IntakeL.spin(directionType::fwd, INTAKE_SPEED_FWD, velocityUnits::pct);
      IntakeR.spin(directionType::fwd, INTAKE_SPEED_FWD, velocityUnits::pct);
      // IntakeL.resetPosition();
      // IntakeR.resetPosition();
    }
    else if(Controller.ButtonDown.pressing()){
      holdingIntakes = false;
      holdingIntakes180 = true;
    }
    else if(Controller.ButtonL1.pressing() && !holdingIntakes){
      // IntakeL.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::pct);
      // IntakeR.spin(directionType::rev, INTAKE_SPEED_REV, velocityUnits::pct);
      holdingIntakes = true;
      holdingIntakes180 = false;
      IntakeL.startRotateFor(directionType::rev, INTAKE_OPEN_POS, rotationUnits::deg,  INTAKE_SPEED_REV, velocityUnits::pct);
      IntakeR.startRotateFor(directionType::rev, INTAKE_OPEN_POS, rotationUnits::deg,  INTAKE_SPEED_REV, velocityUnits::pct);
      IntakeL.setBrake(brakeType::hold);
      IntakeR.setBrake(brakeType::hold);
    } else if(holdingIntakes180){
      IntakeL.spin(directionType::rev, 100, velocityUnits::pct);
      IntakeR.spin(directionType::rev, 100, velocityUnits::pct);
    } else {
      if((IntakeL.isDone() && IntakeR.isDone()) || !holdingIntakes){
        IntakeL.stop();
        IntakeR.stop();
      }
    }

    if(intakePulse > 0){
      intakePulse -= CYCLE_TIME;
    }


    // Delayed trigger buttons code

    // if(Controller.ButtonUp.pressing()){
    //   upBtnFlag = true;
    //   if(directionalButtonCooldown < 0) directionalButtonCooldown = 0;
    // }
    // if(Controller.ButtonRight.pressing()){
    //   rightBtnFlag = true;
    //   if(directionalButtonCooldown < 0) directionalButtonCooldown = 0;
    // }
    // if(Controller.ButtonDown.pressing()){
    //   downBtnFlag = true;
    //   if(directionalButtonCooldown < 0) directionalButtonCooldown = 0;
    // }
    // if(Controller.ButtonLeft.pressing()){
    //   leftBtnFlag = true;
    //   if(directionalButtonCooldown < 0) directionalButtonCooldown = 0;
    // }

    if(Controller.ButtonY.pressing()){
      if(bButtonCooldown >= 0) yBtnFlag = true;
    }
    if(Controller.ButtonB.pressing()){
      bBtnFlag = true;
      if(bButtonCooldown < 0) bButtonCooldown = 0;
    }

    if(directionalButtonCooldown < 0) {}
    else if(directionalButtonCooldown > BUTTON_COOLDOWN_MILLIS){
      if(upBtnFlag){
        if(leftBtnFlag){
          turnToAngle(270 + 45, MACROS_ORTHOGONAL_SPEED);
        }else if(rightBtnFlag){
          turnToAngle(45, MACROS_ORTHOGONAL_SPEED);
        }else{
          turnToAngle(0, MACROS_ORTHOGONAL_SPEED);
        }
      }else if(downBtnFlag){
        if(leftBtnFlag){
          turnToAngle(180 + 45, MACROS_ORTHOGONAL_SPEED);
        }else if(rightBtnFlag){
          turnToAngle(90 + 45, MACROS_ORTHOGONAL_SPEED);
        }else{
          turnToAngle(180, MACROS_ORTHOGONAL_SPEED);
        }
      }else if(leftBtnFlag){
        turnToAngle(270, MACROS_ORTHOGONAL_SPEED);
      }else if(rightBtnFlag){
        turnToAngle(90, MACROS_ORTHOGONAL_SPEED);
      }
      Controller.rumble(".");
      leftBtnFlag = false;
      rightBtnFlag = false;
      upBtnFlag = false;
      downBtnFlag = false;
      directionalButtonCooldown = -1;
    }else{
      directionalButtonCooldown += CYCLE_TIME;
    }

    if(bButtonCooldown < 0) {}
    else if(bButtonCooldown > BUTTON_COOLDOWN_MILLIS){
      if(bBtnFlag){
        if(yBtnFlag){
          if(enableRelativeDriving){
            enableRelativeDriving = false;
            Controller.rumble("--");
          }else{
            enableRelativeDriving = true;
            Controller.rumble("..");
          }
        }else{
          Gyro.resetHeading();
          Controller.rumble("-");
        }
      }

      bBtnFlag = false;
      yBtnFlag = false;
      bButtonCooldown = -1;
    }else{
      bButtonCooldown += CYCLE_TIME;
    }

#if SKILLS
    const int TIME_NOTIF = 50000;
#else
    const int TIME_NOTIF = 75000;
#endif
    if(!timeNotifSounded && t > TIME_NOTIF){ // 10secs left notification
      Controller.rumble(".....");
      timeNotifSounded = true;
    }

    t += CYCLE_TIME;
    vex::this_thread::sleep_for(CYCLE_TIME);
  }
}

void pre_auton(void) {
  vexcodeInit();
  setupRobot();
}


void matchAutonomous(void){

  EncoderL.resetRotation();
  EncoderR.resetRotation();
  EncoderB.resetRotation();

  while(Gyro.isCalibrating()){
    vex::this_thread::sleep_for(20);
  }
}

void liveRemoteAutonomous(void){

  EncoderL.resetRotation();
  EncoderR.resetRotation();
  EncoderB.resetRotation();

  while(Gyro.isCalibrating()){
    vex::this_thread::sleep_for(20);
  }

  Gyro.setHeading(AUTON_STARTING_ROTATION, rotationUnits::deg);
  currRot = (AUTON_STARTING_ROTATION / 180.0f) * M_PI;

  thread rollerThread;
  RollerB.spin(directionType::rev, ROLLER_SPEED_FWD, velocityUnits::pct);
  RollerF.spin(directionType::rev, ROLLER_SPEED_FWD, velocityUnits::pct);
  vex::this_thread::sleep_for(300);
  stopRollers();
  spinIntakes(directionType::fwd);

  // Ball in front of left tower
  smartmove(29, 29, 180 + 45, 5000, false, true, MIN_XY_SPEED, MAX_XY_SPEED);
  smartmove(27, 26, 180 + 45, 500, false, true, MIN_XY_SPEED, 70);

  // Left tower position
  smartmove(23, 23, 180 + 45, 900, true, true);
  vex::this_thread::sleep_for(300);
  rollerThread = spinRollersForAsync(directionType::fwd, 2);
  stopIntakes();
  smartmove(18, 17.5, 180 + 45, 500, true, true);
  rollerThread.join();
  smartmove(24.5, 26, 180 + 90, 3000, false, true, 15, MAX_XY_SPEED, MIN_ROT_SPEED, MAX_ROT_SPEED, ERROR_THRESHOLD_XY * 3); // back up

#if RED_TEAM
  // Go to center tower
  smartmove(49, 52, 0, 3000, false, true, 15, MAX_XY_SPEED, MIN_ROT_SPEED, MAX_ROT_SPEED, ERROR_THRESHOLD_XY * 3);
  // Push ball in from side
  smartmove(59, 52, 0, 700);
#else
  smartmove(51, 52, 0, 3000, false, true, 15, MAX_XY_SPEED, MIN_ROT_SPEED, MAX_ROT_SPEED, ERROR_THRESHOLD_XY * 3);
  smartmove(61, 52, 0, 700);
#endif
  // Back up
  smartmove(57, 36, 0, 1000, false, false, 15, MAX_XY_SPEED, MIN_ROT_SPEED, MAX_ROT_SPEED, ERROR_THRESHOLD_XY * 3);
  

  // MEASURED: Touching right tower is at (121, 21)
  // Start facing right tower
  smartmove(120, 28, 90 + 45, 10000, false, true, 6, 90, 10, 65);
  spinRollers(directionType::fwd);
  spinIntakes(directionType::fwd);
  smartmove(124, 19, 90 + 45, 1000, true);
  // vex::this_thread::sleep_for(200);
  stopIntakes();
  vex::this_thread::sleep_for(1200);
  // spinIntakes(directionType::rev);
  stopIntakes();
  return; // Fix: intakes were descoring a ball, so if we don't move back, it won't pull it out
  smartmove(113, 31, 90 + 45);
  stopIntakes();
  stopRollers();
}

void skillsAutonomous(void) {
  EncoderL.resetRotation();
  EncoderR.resetRotation();
  EncoderB.resetRotation();

  while(Gyro.isCalibrating()){
    vex::this_thread::sleep_for(20);
  }

  Gyro.resetHeading();

  thread rollerThread;

#if AUTON_NOT_PRELOADED
  spinIntakes(directionType::rev);
  vex::this_thread::sleep_for(1000);
#endif

  // Preload needs to start between the bottom front roller and the top back roller
  IntakeL.spin(directionType::fwd, 100, velocityUnits::pct);
  IntakeR.spin(directionType::fwd, 100, velocityUnits::pct);
  smartmove(35.5, 7, 0);

  vex::this_thread::sleep_for(500);
  
  rollerThread = spinRollersForAsync(directionType::fwd, 0.5);
  smartmove(27.7, 25.7, 180 + 45, 5000, false, true, 5, 80, 10, 65);
  smartmove(20.5, 21.5, 0, 900, false, false);
  vex::this_thread::sleep_for(300);

  // On left tower
  rollerThread = spinRollersForAsync(directionType::fwd, 1.5);
  stopIntakes();

  smartmove(18, 17.5, 0, 500, false, false);
  spinIntakes(directionType::rev);
  rollerThread.join();

  RollerF.startRotateFor(directionType::rev, .75, rotationUnits::rev, 100, velocityUnits::pct);
  RollerB.startRotateFor(directionType::fwd, .75, rotationUnits::rev, 100, velocityUnits::pct);
  smartmove(24.5, 26, 180);
  stopIntakes(brakeType::hold);

  // Move left towards center tower
  smartmove(70, 30.5, 180);
  rollerThread = spinRollersForAsync(directionType::fwd, 3.3);
  smartmove(70, 22, 180, 1000);
  rollerThread.join();
  smartmove(70, 34, 180);
  
  smartmove(80, 24, 90, 7000, false, true, 5, 80, 10, 75);
  // Approach ball on the field
  smartmove(87.5, 20, 90);
  spinIntakes(directionType::fwd);
  vex::this_thread::sleep_for(1000);
  rollerThread = spinRollersForAsync(directionType::fwd, .7);
  rollerThread.join();
  spinIntakes(directionType::rev);

  // Start facing right tower
  smartmove(108, 22, 90 + 45, 10000, false, true, 6, 90, 10, 65);
  stopIntakes(brakeType::hold);
  smartmove(119.5, 10.5, 90 + 45, 600);
  rollerThread = spinRollersForAsync(directionType::fwd, 3.2);
  rollerThread.join();
  smartmove(110, 22, 90 + 45);
  stopIntakes();

  smartmove(110, 22, 90 + 45, 5000, false, false);
  // spin first, move 2nd
  spinIntakes(directionType::fwd);
  smartmove(110, 22, 0);

  /*
  // Move backwards to the wall to square up odometry
  moveCardinal(cardinal::reverse, 24, 30, 2000);
  Gyro.resetHeading();
  currRot = 0;
  */

  // At this point, the odometry drifts. Manual adjustment
  // posX -= 1; 
  
  IntakeL.startSpinFor(directionType::rev, AUTON_INTAKE_OPEN_POSITION - 10, rotationUnits::deg);
  IntakeR.startSpinFor(directionType::rev, AUTON_INTAKE_OPEN_POSITION - 10, rotationUnits::deg);
  smartmove(116, 45.5, 0);

  spinIntakes(directionType::fwd);
  vex::this_thread::sleep_for(700);
  rollerThread = spinRollersForAsync(directionType::fwd, 1.8);
  rollerThread.join();
  

  // Approach neutral right
  // Need to move foward/left a tiny bit so it doesn't bump into tower (110 instead of 120)
  smartmove(112, 58, 0, 5000, false, false);
  turnToAngle(90, 85);
  // Dont want to hit left red ball... also bonus if we "accidentally" descore 1 blue!
  spinIntakes(directionType::rev);
  smartmove(121, 59, 90, 1000, false, false);
  rollerThread = spinRollersForAsync(directionType::fwd, 3.14);
  rollerThread.join();

  // Grab ball for center tower
  smartmove(106.5, 67, 90, 1000, false, false);
  turnToAngle(270, 85);
  // smartmove(65, 109, 270);
  spinIntakes(directionType::fwd);
  smartmove(102, 65, 270, 750);
  vex::this_thread::sleep_for(500);
  rollerThread = spinRollersForAsync(directionType::fwd, 1.3);
  rollerThread.join();

  // Poke ball out of center tower (with left arm)
  smartmove(102.5, 70.5, 270, 800);
  stopIntakes();
  smartmove(88, 70.5, 270, 900);
  smartmove(102, 70.5, 270, 900);
  // Try again just in case
  smartmove(88, 68.5, 270, 900);
  smartmove(102, 68.5, 270, 900);
  // Retreat
  spinIntakes(directionType::rev);
  vex::this_thread::sleep_for(300);
  rollerThread = spinRollersForAsync(directionType::rev, .4);
  rollerThread.join();
  smartmove(104, 67, 270);
  smartmove(86, 67, 270, 800);
  rollerThread = spinRollersForAsync(directionType::fwd, 3.5);
  rollerThread.join();


  // Back up
  smartmove(113, 67, 270);
  spinIntakes(directionType::fwd);
  turnToAngle(0, 85);
  IntakeL.startSpinFor(directionType::rev, AUTON_INTAKE_OPEN_POSITION + 10, rotationUnits::deg);
  IntakeR.startSpinFor(directionType::rev, AUTON_INTAKE_OPEN_POSITION + 10, rotationUnits::deg);

  // Go for ball on blue side of field
  smartmove(108.5, 95, 0);
  spinIntakes(directionType::fwd);
  vex::this_thread::sleep_for(1000);
  rollerThread = spinRollersForAsync(directionType::rev, .4);
  rollerThread.join();
  spinIntakes(directionType::rev);

  // Go to blue right tower
  smartmove(125, 116, 45, 2000);
  stopIntakes(brakeType::hold);
  rollerThread = spinRollersForAsync(directionType::fwd, 3);
  rollerThread.join();
  smartmove(117, 107, 45);

  // Grab ball on side of wall
  smartmove(117, 102, 90);
  smartmove(134, 102, 90, 800);
  rollerThread = spinRollersForAsync(directionType::fwd, 2);
  rollerThread.join();

  // Approach blue mid
  smartmove(74, 98, 0);
  rollerThread = spinRollersForAsync(directionType::fwd, 4);
  smartmove(74, 111, 0, 1100);
  rollerThread.join();

  // Back up
  smartmove(75, 98, 0);

}

int main() {
  std::cout.precision(3);
  pre_auton();


  task odomTask( odometryTaskCallback );


  if(SKILLS){
    Competition.autonomous(skillsAutonomous);
  }else if(LIVE_REMOTE){
    Competition.autonomous(liveRemoteAutonomous);
  }else{
    Competition.autonomous(matchAutonomous);
  }
  Competition.drivercontrol(usercontrol);

  while (true) {
    vex::this_thread::sleep_for(100);
  }
}
