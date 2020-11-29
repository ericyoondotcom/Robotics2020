/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       kensukeshimojo                                            */
/*    Created:      Sun Nov 15 2020                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// intakeR              motor         14              
// intakeL              motor         11              
// backRollers          motor         15              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  while(1){

    
    
    if(Controller1.ButtonR1.pressing()){
      intakeL.spin(reverse,30,rpm);
      intakeR.spin(reverse,30,rpm);
    }
    else if(Controller1.ButtonR2.pressing()){
      intakeL.spin(forward,80,rpm);
      intakeR.spin(forward,80,rpm);
    }
    else if(Controller1.ButtonUp.pressing()){
      intakeL.spin(reverse,200,rpm);
    }
    else {
      //intakeL.setStopping(brake);
      intakeR.setStopping(brake);
      intakeL.stop();
      intakeR.stop();
    }
    
    if(Controller1.ButtonL1.pressing()){
      backRollers.spin(forward,183,rpm);
    }
    else if(Controller1.ButtonL2.pressing()){
      backRollers.spin(reverse,170,rpm);
  }else{
    backRollers.stop();
  }
  
  }
}
