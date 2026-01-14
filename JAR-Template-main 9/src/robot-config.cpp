#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors

motor rightFront = motor(PORT7,ratio18_1, false); //done
motor rightBack = motor(PORT3, ratio18_1, false); //done
motor rightMid = motor(PORT12, ratio18_1, false); //done
motor leftFront = motor(PORT14 , ratio18_1, true); //done
motor leftBack = motor(PORT6, ratio18_1, true); //done
motor leftMid = motor(PORT1, ratio18_1, true); //done

controller Controller1 = controller(primary);

motor Intake = motor(PORT21, ratio36_1, true); 

motor firstStage = motor(PORT21,ratio18_1, true); //the front first stage of the intake

motor secondstage = motor(PORT16,ratio18_1, true); // the second stage (the one that pushes balls into the bucket)
motor scorer = motor(PORT5,ratio18_1, true); // the one that determines whether balls go into low or high goal
optical opticalsorter = optical(PORT10);
inertial Inertial = inertial(PORT9); 
digital_out descore = digital_out(Brain.ThreeWirePort.E);

digital_out MOGO = digital_out(Brain.ThreeWirePort.A);
distance colordistance = distance(PORT20);
motor Upper = motor(PORT15, ratio18_1, true);
digital_out mogo = digital_out(Brain.ThreeWirePort.B);
optical colorsorter1 = optical(PORT20);  
optical colorsorter2 = optical(PORT20);
motor WallStakes = motor(PORT15, ratio18_1, true);
rotation rotationSensor = rotation(PORT15,false);
digital_out Puller = digital_out(Brain.ThreeWirePort.C);
digital_out DoinkerRight = digital_out(Brain.ThreeWirePort.H);
digital_out DoinkerLeft = digital_out(Brain.ThreeWirePort.E);
digital_out Doinker = digital_out(Brain.ThreeWirePort.G); //change later
digital_out sorter = digital_out(Brain.ThreeWirePort.G);
digital_out Tipper = digital_out(Brain.ThreeWirePort.G);
digital_out Matchloader = digital_out(Brain.ThreeWirePort.H);
digital_out indexer = digital_out(Brain.ThreeWirePort.A);
digital_out bottomDescore = digital_out(Brain.ThreeWirePort.F);

digital_out hardStop = digital_out(Brain.ThreeWirePort.A); //change port later





// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}
