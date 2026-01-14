#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
ZERO_TRACKER_ODOM,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(leftFront,leftMid,leftBack),

//Right Motors:
motor_group(rightFront,rightMid,rightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT9,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
3.25,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value is 1:
0.8,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
3,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
2.75,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
-2,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
1,

//Sideways tracker diameter (reverse to make the direction switch):
-2.75,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
0

);


//AntiJam code stopper variable

bool codestopper129 = false;
bool matchloaderdown1 = false;

void arcadeDrive(){ 
 
 float throttle = deadband(controller(primary).Axis3.value(), 1);
 float turn = deadband(controller(primary).Axis1.value(), 12);
 leftFront.spin(fwd, to_volt(throttle+turn), percent);
 leftMid.spin(fwd, to_volt(throttle+turn), percent);
 leftBack.spin(fwd, to_volt(throttle+turn), percent);
 rightFront.spin(fwd, to_volt(throttle-turn), percent);
 rightMid.spin(fwd, to_volt(throttle-turn), percent);
 rightBack.spin(fwd, to_volt(throttle-turn), percent);

 }


void ThrowAwayBlue() {

 color blue = color((0, 0, 255));

 if (colorsorter1.color() == blue) {
 Controller1.rumble("...");
 sorter.set(true);
 wait(1000,msec);
 sorter.set(false);
 //Intake.stop(hold);

 }
}



void ThrowAwayRed() {

 colorsorter1.setLightPower(100,percent);

 if (colorsorter1.color() == red) {
 Controller1.rumble("...");
 sorter.set(true);
 wait(1000,msec);
 sorter.set(false);
 //Intake.stop(hold);

 }
}

void driveCurved(double targetPosition, double speed, int curve, double kp) {
 // reset motor positions
 leftFront.resetPosition();
 leftMid.resetPosition();
 leftBack.resetPosition();
 rightFront.resetPosition();
 rightMid.resetPosition();
 rightBack.resetPosition();

 //stop all motors
 leftFront.stop(brake);
 leftMid.stop(brake);
 leftBack.stop(brake);
 rightFront.stop(brake);
 rightMid.stop(brake);
 rightBack.stop(brake);

 double realTarget = (targetPosition * 15); //scales up the target so inputted numbers can be lower
 double error = 0.0;
 double output = 0.0;


 double tolerance = 5.0;
 double speedLimit = (speed / 200);


 while (true) {


 // error
 double currentPosition = (leftFront.position(degrees) + rightFront.position(degrees)) / 2;
 error = realTarget - currentPosition;




 output = kp * error;


 //stops output from overheating motors
 if (output > 100.0) {
 output = 100.0;
 } else if (output < -100.0) {
 output = -100.0;
 }


 // adjust output for curve
 double leftSpeed = output;
 double rightSpeed = output;


 // positive curve = right motors slower
 if (curve > 0) {
 rightSpeed = output - (output * curve / 100);
 }
 // negative curve = left motors slower
 else if (curve < 0) {
 leftSpeed = output - (output * (-curve) / 100);
 }


 // gives max speed
 leftSpeed *= speedLimit;
 rightSpeed *= speedLimit;


 // spins motors
 leftFront.spin(forward, leftSpeed, percent);
 leftMid.spin(forward, leftSpeed, percent);
 leftBack.spin(forward, leftSpeed, percent);
 rightFront.spin(forward, rightSpeed, percent);
 rightMid.spin(forward, rightSpeed, percent);
 rightBack.spin(forward, rightSpeed, percent);


 // checks if error is less than tolerace
 if (fabs(error) < tolerance) {
 // Stop all motors
 leftFront.stop();
 leftMid.stop();
 leftBack.stop(brake);
 rightFront.stop();
 rightMid.stop();
 rightBack.stop(brake);
 wait(20,msec);
 break; 
 }


 wait(20,msec);
}
}



//test wallstakes macros using IME's

void WallStakesMacro(double targetPosition){

 double kp = 0.1;

 WallStakes.resetPosition();
 WallStakes.setPosition(0,degrees);

 double realTarget = (targetPosition*15);
 double error = 0.0;
 double output = 0.0;


 double tolerance = 1.0;
 double speedLimit = 50;


 while (true) {
 // get the error
 double currentPosition = WallStakes.position(degrees); // motor positions
 error = realTarget - currentPosition;


 // output
 output = kp * error;


 if (output > 100.0) {
 output = 100.0;
 } else if (output < -100.0) {
 output = -100.0;
 }

 output *= speedLimit; 


 // spins motors
 WallStakes.spin(forward, output, percent);

 // checks to see if within tolerance
 if (fabs(error) < tolerance) {
 // Stop all motors

 wait(20,msec); // wait so commands have time to work
 break; // 
 }


 wait(20,msec); 
 }

}

//Old P function, unused now

void driveForward(double targetPosition,double speed,double kp){
 // reset the motor positions
 leftFront.resetPosition();
 leftMid.resetPosition();
 leftBack.resetPosition();
 rightFront.resetPosition();
 rightMid.resetPosition();
 rightBack.resetPosition();


 leftFront.stop(brake);
 leftMid.stop(brake);
 leftBack.stop(brake);
 rightFront.stop(brake);
 rightMid.stop(brake);
 rightBack.stop(brake);

 //double kp = 0.7;


 double realTarget = (targetPosition*15);
 double error = 0.0;
 double output = 0.0;


 double tolerance = 5.0;
 double speedLimit = (speed/200); //max speed cap is 50 percent


 while (true) {
 //error
 double currentPosition = (leftFront.position(degrees) + rightFront.position(degrees)) / 2; // dt motor positions
 error = realTarget - currentPosition;



 output = kp * error;


 //limits output to |100|
 if (output > 100.0) {
 output = 100.0;
 } else if (output < -100.0) {
 output = -100.0;
 }




 output *= speedLimit; // multiplies value by speed limit to get the right speed


 // spins motors
 leftFront.spin(forward, output, percent);
 leftMid.spin(forward, output, percent);
 leftBack.spin(forward, output, percent);
 rightFront.spin(forward, output, percent);
 rightMid.spin(forward, output, percent);
 rightBack.spin(forward, output, percent);
 // checks to see if within tolerance
 if (fabs(error) < tolerance) {
 // Stop all motors
 leftFront.stop();
 leftMid.stop();
 leftBack.stop(brake); //only back wheels stopped to prevent them from bouncing up
 rightFront.stop();
 rightMid.stop();
 rightBack.stop(brake);
 wait(20,msec); // wait so commands have time to work
 break; 
 }


 wait(20,msec); 

 }
}


vex::task antiJamCode() {

 while(1){

 wait(10,msec);

 Controller1.rumble("...");
 Controller1.Screen.clearScreen();
 Controller1.Screen.setCursor(1, 1);

 Controller1.Screen.print(Upper.current());

 if (Upper.current()>2) {
 
 wait(500,msec);

 if (Upper.current()>2) {

 Controller1.Screen.print(Upper.current());
 Controller1.rumble("...");
 Upper.spin(reverse,100,percent);
 wait(250,msec);
 Upper.spin(forward,100,percent);

 }
 }
 }
 }



 double intakestopthreesec = 0.5; //variable to stop antijam from working in corners (current spike always triggers it)


vex::task tasksToggle() {


 int count = 1;

 while(1){

if (matchloaderdown1 == true){

 matchloaderdown1 = false;

 wait(1000,msec);
 Matchloader.set(true);

}

 }

 return(0);
 
}

vex::task ColorSortRed() {

 while(1){

 colorsorter1.setLightPower(100,percent);
 colorsorter2.setLightPower(100,percent);
 Controller1.Screen.clearScreen();
 Controller1.Screen.setCursor(1, 1);

 //Controller1.Screen.print(Intake.current()); //debugging
 
 
 Controller1.Screen.print(colorsorter1.color());
 
 if (Intake.current()>2.5 and intakestopthreesec==0.5) {
 wait(500,msec); //to prevent antijams for momentary high voltage.
 if (Intake.current()>2.5) {
 
 Controller1.Screen.print(Intake.current());
 Intake.spin(reverse,100,percent);
 wait(250,msec);
 Intake.spin(forward,100,percent);
 wait(250,msec);
 
 }
 }
 
 if (colorsorter1.color() == red and colorsorter1.isNearObject() or colorsorter2.color() == red and colorsorter2.isNearObject()) {
 
 Intake.stop();
 sorter.set(true);
 wait(100,msec);

 Intake.spin(forward,100,percent);
 
 wait(1000,msec);
 
 sorter.set(false);
 
 
 Controller1.rumble("..."); //lets driver know that the ring is sorting
 
 }
 
 /*
 else if (intakestopthreesec > 0.5) {
 
 wait(3000,msec);
 intakestopthreesec = 0.5;
 }
 */
 }
 
}

vex::task ColorSortBlue() {

 while(1){

 colorsorter1.setLightPower(100,percent);
 colorsorter2.setLightPower(100,percent);
 Controller1.Screen.clearScreen();
 Controller1.Screen.setCursor(1, 1);
 //Controller1.Screen.print(Intake.current());

 Controller1.Screen.print(colorsorter1.color());

 if (Intake.current()>2.4 and intakestopthreesec==0.5) {
 Controller1.rumble("...");
 wait(500,msec);
 if (Intake.current()>2.5) {

 Controller1.Screen.print(Intake.current());
 Intake.spin(reverse,100,percent);
 wait(250,msec);
 Intake.spin(forward,100,percent);
 wait(250,msec);

 }
 }

 if (colorsorter1.color() == blue and colorsorter1.isNearObject() or colorsorter2.color() == blue and colorsorter2.isNearObject()) {

 Intake.stop();
 sorter.set(true);
 wait(100,msec);
 Intake.spin(forward,100,percent);
 wait(1000,msec);

 sorter.set(false);

 Controller1.rumble("..."); //lets me know that ring is sorting

 }
 }

}





int driveFunction() {
int count = 0;

 while(true) {
/*
    if (Controller1.ButtonX.pressing()){
        rightFront.stop(hold);
        rightMid.stop(hold);
        rightBack.stop(hold);
        leftFront.stop(hold);
        leftMid.stop(hold);
        leftBack.stop(hold);
    }

 /*tank drive code
 Controller1.rumble("...");
 float leftthrottle = deadband(controller(primary).Axis3.value(), 5); //deadband decrease unwanted controller drift
 float rightthrottle = deadband(controller(primary).Axis2.value(), 5); //deadband decrease unwanted controller drift
 leftFront.spin(fwd, to_volt(leftthrottle), volt);
 leftMid.spin(fwd, to_volt(leftthrottle), volt);
 leftBack.spin(fwd, to_volt(leftthrottle), volt);
 rightFront.spin(fwd, to_volt(rightthrottle), volt);
 rightMid.spin(fwd, to_volt(rightthrottle), volt);
 rightBack.spin(fwd, to_volt(rightthrottle), volt);

*/
//chassis.control_arcade();
chassis.control_tank();
if(Controller1.ButtonLeft.pressing()){

  secondstage.spin(forward,100,percent);
  scorer.spin(reverse,20,percent);


}

else if(Controller1.ButtonRight.pressing()){

  secondstage.spin(forward,100,percent);
  scorer.spin(reverse,50,percent);


}
 this_thread::sleep_for(10);
 }

 return(0);
}





int matchloaderFunction(){

 int count = 1;

 while(true){
Controller1.rumble("");
 this_thread::sleep_for(10);
 if(Controller1.ButtonUp.pressing()){

 //DoinkerRight.set(true); //change before every match depending on the alliance color
 Matchloader.set(true);
 Controller1.rumble("...");
 wait(10,msec);

 }
 else if(Controller1.ButtonDown.pressing()){

 Matchloader.set(false);
 Controller1.rumble("...");
 wait(10,msec);
 }
 }

return(0);

}

int toggleIntakeFunction() {


 // XEnabled = true;

 bool middleEnabled = false;
 bool longEnabled = false;
 bool lowEnabled = false;
 bool basketEnabled = false;
 bool testEnabled = false;
 bool test2 = false;
 bool testdown = false;
 bool testHold = false;
 bool XEnabled = false;
 bool upEnabled = false; 

 bool R1Pressed = false;
 bool R2Pressed = false;
 bool L1Pressed = false;
 bool L2Pressed = false;
 bool APressed = false;
 bool BPressed = false;
 bool DownPressed = false;
 bool XPressed = false;
 bool YPressed = false;
 bool UpPressed = false; 

indexer.set(true);
 while (true) {

   if (Controller1.ButtonR1.pressing() and R1Pressed == false) {
     R1Pressed = true;
     if (middleEnabled == true) {
       firstStage.stop();
       secondstage.stop();
       scorer.stop(hold);
       middleEnabled = false;
     } else {
       secondstage.spin(forward,100,percent);

       middleEnabled = true;
     }
   } 

   if (Controller1.ButtonR2.pressing() and R2Pressed == false) {
     R2Pressed = true;
     if (longEnabled == true) {
       firstStage.stop();
       secondstage.stop();
       scorer.stop(hold);
       longEnabled = false;
     } else {
       secondstage.spin(reverse,40,percent);
       scorer.spin(reverse,100,percent);
       longEnabled = true;
     }
   } 

   if (Controller1.ButtonL1.pressing() and L1Pressed == false) {
     L1Pressed = true;
     if (lowEnabled == true) {
       scorer.stop(hold);
       secondstage.stop();
       lowEnabled = false;
     } else {
       secondstage.spin(forward,100,percent);
       scorer.spin(forward,100,percent);
       lowEnabled = true;
     }
   } 

   if (Controller1.ButtonDown.pressing() and L2Pressed == false) {
     L2Pressed = true;
     if (basketEnabled == true) {
       firstStage.stop();
       secondstage.stop();
       scorer.stop(hold);
       basketEnabled = false;
     } else {
       secondstage.spin(forward,100,percent);
       scorer.spin(forward,100,percent); 
       wait(350,msec);
       scorer.stop(hold);
       secondstage.stop();
       basketEnabled = true;
     }
   }

   if (Controller1.ButtonA.pressing() and APressed == false) {
     APressed = true;
     if (testEnabled == true) {
       Matchloader.set(false);
       wait(300,msec);
       testEnabled = false;
     } else {
       Matchloader.set(true);
       wait(300,msec);
       testEnabled = true;
     }
   }

   if (Controller1.ButtonB.pressing() and BPressed == false) {
     BPressed = true;
     if (test2 == true) {
       indexer.set(true);
     //  hardStop.set(true);
       scorer.stop();
       secondstage.stop();
       wait(250,msec);
       test2 = false;
     } else {
       indexer.set(false);
      // hardStop.set(false);

      scorer.spin(reverse,100,percent);
      secondstage.spin(forward,100,percent);
       wait(250,msec);

       test2 = true;
     }
   }

   if (Controller1.ButtonL2.pressing() and DownPressed == false) {
     DownPressed = true;
     if (testdown == true) {

       bottomDescore.set(false);
       scorer.stop();
       secondstage.stop();

       wait(250,msec);
       testdown = false;
     } else {

       bottomDescore.set(true);
       //scorer.spin(reverse,100,percent);
       //secondstage.spin(forward,100,percent);
       wait(250,msec);
       testdown = true;
     }
   }

   if (Controller1.ButtonX.pressing() && XPressed == false) {
     XPressed = true;
     if (XEnabled == true) {
      descore.set(false);
       XEnabled = false;
     } else {
      descore.set(true);
       XEnabled = true;
     }
   }


   if (Controller1.ButtonUp.pressing() && UpPressed == false) {
     UpPressed = true;

     if (upEnabled == true) {

       scorer.stop();
       secondstage.stop();
       middleEnabled = false;
lowEnabled = false;


       upEnabled = false;
     } else {

      scorer.stop();
       secondstage.stop();
              middleEnabled = false;
lowEnabled = false;
       

       upEnabled = true;
     }
   }
   // ------------------------------------------

   if (Controller1.ButtonR1.pressing() == false &&
       Controller1.ButtonR2.pressing() == false &&
       Controller1.ButtonL1.pressing() == false &&
       Controller1.ButtonL2.pressing() == false &&
       Controller1.ButtonUp.pressing() == false &&   
       Controller1.ButtonA.pressing() == false &&   
       Controller1.ButtonB.pressing() == false &&   
       Controller1.ButtonUp.pressing() == false &&   
       Controller1.ButtonX.pressing() == false) {

     R1Pressed = false;
     R2Pressed = false;
     L1Pressed = false;
     L2Pressed = false;
     APressed = false;
     BPressed = false;
     DownPressed = false;
     XPressed = false;
     UpPressed = false;
   }

   this_thread::sleep_for(10);
 }

 return 0;
}







int tempFunction(){

int count = 1;

 while(true){

 this_thread::sleep_for(10);

 if (rightMid.temperature(temperatureUnits::fahrenheit) > 150){
 Controller1.rumble("...");
 Controller1.Screen.print("STOP DRIVING!!!");
 }
 else if(rightMid.temperature(temperatureUnits::fahrenheit) < 150){

 }
 else{

 }
 }

return(0);

}

void TempCode(){

 if (rightMid.temperature(temperatureUnits::fahrenheit) > 130){
 Controller1.rumble("...");
 Controller1.Screen.print("STOP DRIVING!!!");
 }
 else if(rightMid.temperature(temperatureUnits::fahrenheit) < 120){
 }
 else{

 }
}





//auton selector
int current_auton_selection = 0;
bool auto_started = false;


void pre_auton(void) {
vexcodeInit();


Inertial.calibrate();

while (Inertial.isCalibrating() == true){
Controller1.rumble("...");
wait(500,msec);
}

current_auton_selection = 0;


default_constants();

while(auto_started == false){
//Brain.Screen.clearScreen();
switch(current_auton_selection){
case 0:

 Brain.Screen.printAt(50, 50, "");

break;

case 1:

 Brain.Screen.printAt(50, 50, "");
 wait(1000,msec);

case 2:

 Brain.Screen.printAt(50, 50, "");

break;

case 3:

 Brain.Screen.printAt(50, 50, "");

break;


case 4:

 Brain.Screen.printAt(50, 50, "");

break;

case 5:

 Brain.Screen.printAt(50, 50, "");

break;

case 6:

 Brain.Screen.printAt(50, 50, "");

break;

case 7:

 Brain.Screen.printAt(50, 50, "");

break;

case 8:

 Brain.Screen.printAt(50, 50, "");

break;

case 9:

 Brain.Screen.printAt(50, 50, "");

break;

case 10:

 Brain.Screen.printAt(50, 50, "");

break;

case 11:

 Brain.Screen.printAt(50, 50, "");

break;

case 12:

 Brain.Screen.printAt(50, 50, "");

break;

case 13:

Brain.Screen.printAt(50, 50, "");

break;


}

if(Brain.Screen.pressing()){

 while(Brain.Screen.pressing()) {}

 Brain.Screen.clearScreen();
 current_auton_selection ++;
} 

else if (current_auton_selection == 14){
 current_auton_selection = 0;
}
task::sleep(10);
 }

}








void autonomous(void) {
auto_started = true;
switch(current_auton_selection){
case 0: //SOLO AWP

descore.set(true);
indexer.set(true);
secondstage.spin(forward,100,percent);

chassis.drive_distance(12.5);

Matchloader.set(true);

secondstage.spin(forward,100,percent);

chassis.turn_to_angle(90,12,10,10,1000);

chassis.drive_distance(3,90);


chassis.drive_distance(30,90,4,12,1,100,700);




chassis.drive_distance(-30,90,12,12,1,100,500); //replace other one if this works


secondstage.spin(forward,100,percent);
scorer.spin(forward,100,percent);



chassis.drive_distance(-30,90,3,3,1,100,600);

wait(700,msec);

Matchloader.set(false);

scorer.stop(hold);



chassis.turn_to_angle(190);


secondstage.spin(forward,100,percent);

//chassis.drive_kp = 5; //delete if needed


chassis.drive_distance(15.4,180);

chassis.set_coordinates(0,0,0);
chassis.drive_to_point(0,3,6,12,12);
Matchloader.set(true);
chassis.drive_distance(3.5,0);


chassis.set_coordinates(0,0,180);
chassis.turn_to_angle(135);




secondstage.stop();
scorer.stop(hold);

indexer.set(false);
//hardStop.set(false);







chassis.drive_distance(-6,135,12,12,1,100,1000);

secondstage.spin(forward,100,percent);
scorer.spin(reverse,100,percent);

chassis.drive_distance(-30,135,2,12,1,100,500);



wait(700,msec);
secondstage.stop(hold);
scorer.stop(hold);

////
////
////
////
////


//middle goal over

//chassis.set_coordinates(0,0,0);
//chassis.drive_to_point(-0.3,15.7,4,12,12);

//

chassis.drive_distance(18,130);

//chassis.drive_distance(17.1,135);
secondstage.spin(forward,100,percent);

hardStop.set(true);

indexer.set(true);

//chassis.turn_to_angle(90,12,5,50,1000);

chassis.turn_to_angle(90,12,10,10,1000);
chassis.drive_distance(30,90,4,12,1,100,1400);



chassis.drive_distance(-30,90,12,12,1,100,500); //replace other one if this works


secondstage.spin(forward,100,percent);
scorer.spin(forward,100,percent);

chassis.drive_distance(-30,87,12,12,1,100,200); //replace other one if this works

chassis.drive_distance(-30,87,3,3,1,100,500);




//chassis.drive_kp = 5; //delete if needed

/*
chassis.drive_distance(15.9,145);
indexer.set(false);
scorer.stop(hold);

indexer.set(false);
hardStop.set(true);

chassis.turn_to_angle(90,12,5,10,3000);
*/

//here
/*

secondstage.spin(forward,100,percent);
scorer.stop(hold);

chassis.drive_kp = 5;
chassis.drive_distance(3);

chassis.drive_kp = 1.4;

chassis.drive_distance(30,90,3,12,1,100,1000);



chassis.drive_kp = 10;

chassis.drive_distance(-9,90,12,12);

secondstage.spin(forward,100,percent);
scorer.spin(forward,100,percent);


chassis.drive_kp = 1.4;

chassis.drive_distance(-30,90,8,3,1,100,300);

chassis.drive_distance(1.1);

//here
*/

/*
scorer.stop(hold);

chassis.drive_distance(-30,90,8,3,1,100,500);

secondstage.spin(forward,100,percent);
scorer.spin(forward,100,percent);

chassis.drive_distance(1.1);

Matchloader.set(false);


*/









break;


case 3: //Left Side Descore Auton


chassis.drive_distance(-14.5,330);
scorer.spin(forward,100,percent);
wait(500,msec);
scorer.stop();
secondstage.spin(forward,100,percent);
chassis.drive_distance(12,265,4,3);

secondstage.setVelocity(100,percent);
//secondstage.spinFor(reverse,100,degrees,false);
chassis.drive_distance(4,225);
secondstage.spin(reverse,100,percent);
wait(180,msec);
secondstage.spin(forward,100,percent);

//secondstage.spinFor(reverse,50,degrees);
secondstage.spin(forward,100,percent);
scorer.setVelocity(100,percent);

//secondstage.spin(forward,100,percent);

chassis.drive_distance(5.1,225,4,12);// change to 219 if needed
wait(250,msec);


chassis.drive_distance(-5.7,205,12,12);
scorer.spin(forward,10,percent);

chassis.drive_distance(-8,100,12,12);
scorer.stop();
chassis.turn_to_angle(0,10,2,50,500);

secondstage.spin(forward,100,percent);


chassis.drive_distance(-15.4,0,6,12,1,150,300);

chassis.drive_distance(-20,0,3,12,1,150,500);



secondstage.spin(forward,100,percent);
scorer.spin(forward,100,percent);

//chassis.drive_distance(-20,0,3,12,1,150,1500);
secondstage.spin(reverse,100,percent);
wait(150,msec);
secondstage.spin(forward,100,percent);
wait(1000,msec);
//wait(1500,msec);

scorer.spinFor(reverse,200,degrees,false);

Matchloader.set(true);

chassis.drive_distance(8);
chassis.drive_distance(30,180,3,12,1,100,1200); 


chassis.drive_distance(-9,180,12,12);


scorer.spin(forward,100,percent);

chassis.drive_distance(-30,180,8,3,1,100,500);


Matchloader.set(false);

chassis.drive_distance(-30,180,3,3,1,100,2500);


break;

case 2: //Right Side 7 Ball Descore Auton


scorer.stop(hold);
secondstage.spin(forward,100,percent);


hardStop.set(true);

chassis.drive_distance(6,35,12,10);
Matchloader.set(true);
chassis.drive_distance(5,50,5,2);

//chassis.drive_distance(13.3,50,5,2);

Matchloader.set(true);

chassis.turn_to_angle(140,12,5,1,3000);



chassis.drive_distance(12.7,150,12,12); //tried 0 settle time change to 13.6

chassis.turn_to_angle(180);
chassis.drive_distance(30,180,3,12,1,100,1200); 



//chassis.drive_distance(15.3,155,12,12); //tried 0 settle time change to 13.6


//chassis.drive_distance(30,180,2,12,1,100,1500); 

chassis.drive_distance(-30,180,12,12,1,100,500); //replace other one if this works


scorer.spin(forward,100,percent);

chassis.drive_distance(-30,180,4,3,1,100,500);


Matchloader.set(false);




wait(1200,msec);

chassis.turn_to_angle(180);
chassis.set_coordinates(0,0,0);
scorer.stop();
chassis.drive_distance(3);



chassis.drive_distance(-3,50,12,12);

chassis.drive_distance(-9,340);

chassis.turn_to_angle(10);




/*
chassis.drive_distance(10,70);

chassis.turn_to_angle(180);
*/

break;


case 1: //Left Side 4+5

hardStop.set(true);

secondstage.spin(forward,100,percent);

scorer.spin(reverse,10,percent);

chassis.drive_distance(13.6,310,5,2);

chassis.drive_distance(7.7,300,5,2,1,10,3000);



wait(500,msec);




chassis.drive_distance(-9.8,285);



secondstage.spinFor(reverse,50,degrees,false);

chassis.turn_to_angle(225);

secondstage.stop();
scorer.stop();

scorer.spin(reverse,100,percent);

indexer.set(true);
hardStop.set(false);



Matchloader.set(true);
scorer.stop(hold);

chassis.drive_distance(-30,225,5,12,1,100,200);
chassis.drive_distance(-30,225,2,12,1,100,500);

secondstage.spin(forward,100,percent);
scorer.spin(reverse,100,percent);

chassis.drive_distance(-30,225,2,12,1,100,500);



wait(600,msec);
secondstage.stop(hold);
scorer.stop(hold);



////
////
////
////
////


//middle goal over

//chassis.set_coordinates(0,0,0);
//chassis.drive_to_point(-0.3,15.7,4,12,12);

//

chassis.drive_distance(18.1,220);

//chassis.drive_distance(17.1,135);
secondstage.spin(forward,100,percent);

hardStop.set(true);
indexer.set(false);


//chassis.turn_to_angle(90,12,5,50,1000);

chassis.turn_to_angle(180,12,10,10,1000);
chassis.drive_distance(30,180,4,12,1,100,1200);



chassis.drive_distance(-30,180,12,12,1,100,500); //replace other one if this works


secondstage.spin(forward,100,percent);
scorer.spin(forward,100,percent);


chassis.drive_distance(-30,180,12,12,1,100,200); //replace other one if this works

chassis.drive_distance(-30,180,3,3,1,100,500);

wait(1200,msec);
chassis.turn_to_angle(180);
chassis.set_coordinates(0,0,0);

chassis.drive_distance(3);

scorer.stop();

chassis.drive_distance(-2.8,50,12,12);

chassis.drive_distance(-9,340);

chassis.turn_to_angle(10);





break;







case 4: //
secondstage.spin(forward,100,percent);
Matchloader.set(true);
chassis.drive_distance(10.5);


chassis.turn_to_angle(270);

chassis.drive_distance(30,270,3,12,1,100,1200); 

chassis.drive_distance(-9,270,12,12);



chassis.drive_distance(-30,270,8,3,1,100,500);

scorer.spin(forward,100,percent);


Matchloader.set(false);

chassis.drive_distance(-30,270,3,3,1,100,2500);


break;




case 5: //


chassis.drive_distance(1.1);

chassis.turn_to_angle(180);
chassis.set_coordinates(0,0,0);
chassis.drive_to_point(1,7,8,12,12);
Matchloader.set(true);
chassis.drive_to_point(0,10,8,12,12);
Matchloader.set(false);
chassis.drive_to_point(0,20);
Matchloader.set(true);





break;

case 6: // 


break;


case 7: //


/*
chassis.set_coordinates(0,0,0);

chassis.drive_to_point(0,5,8,12,12);

Matchloader.set(true);

chassis.drive_to_point(0,25,0,12,12);

Matchloader.set(false);

*/
//wing part to put into 7 ball rush
chassis.drive_distance(3);


chassis.drive_distance(-3,50,12,12);

chassis.drive_distance(-9,340);

chassis.turn_to_angle(10);


break;


case 8: //




break;


case 9: //Right Side Descore Old

scorer.stop(hold);
secondstage.spin(forward,100,percent);

chassis.drive_distance(13.3,50,5,3);




chassis.drive_distance(-11.8,250);

/*
secondstage.spin(reverse,50,percent);
wait(250,msec);
secondstage.spin(forward,100,percent);
*/

chassis.turn_to_angle(180);

Matchloader.set(true);
secondstage.spin(reverse,100,percent);
wait(180,msec);
secondstage.spin(forward,100,percent);



chassis.drive_distance(5.5);//delete if changing back

secondstage.spin(forward,100,percent);

chassis.drive_distance(25,180,3,4,1,100,700);


chassis.drive_distance(-15,180,8,12,1,100,700);


scorer.spin(forward,100,percent);


chassis.drive_distance(-20,180,3,12,1,150,500); 
secondstage.spin(reverse,100,percent);
wait(250,msec);
secondstage.spin(forward,100,percent);

wait(2500,msec);

descore.set(true);



chassis.turn_to_angle(260);

scorer.stop();
secondstage.stop();

descore.set(false);

chassis.drive_distance(4);


chassis.drive_distance(-5.6,215);




chassis.drive_distance(-5,180);
chassis.turn_to_angle(180);

chassis.drive_distance(-30,180,3,12,1,100,9000);




break;


case 10: //1+8 Old

chassis.drive_distance(-14.5,330);
scorer.spin(forward,10,percent);
wait(500,msec);
scorer.stop();
secondstage.spin(forward,50,percent);
chassis.drive_distance(12,265,4,3);

secondstage.setVelocity(100,percent);
//secondstage.spinFor(reverse,100,degrees,false);
chassis.drive_distance(4,225);

//secondstage.spinFor(reverse,50,degrees);
secondstage.spin(forward,50,percent);
scorer.setVelocity(100,percent);

//secondstage.spin(forward,100,percent);

chassis.drive_distance(5.1,225,4,12);// change to 219 if needed
wait(750,msec);


chassis.drive_distance(-5.7,205,12,12);

chassis.drive_distance(-8,100,12,12);
scorer.stop();
chassis.turn_to_angle(0,10,2,50,500);

secondstage.spin(forward,100,percent);


chassis.drive_distance(-15.4,0,12,12,1,150,400);

secondstage.spin(forward,100,percent);
scorer.spin(forward,100,percent);

chassis.drive_distance(-20,0,3,12,1,150,500);

wait(1500,msec);



secondstage.spin(forward,100,percent);
scorer.spin(forward,100,percent);


scorer.spinFor(reverse,200,degrees,false);

Matchloader.set(true);

chassis.drive_distance(8);
chassis.drive_distance(30,180,3,12,1,100,1200); 


chassis.drive_distance(-9,180,12,12);


scorer.spin(forward,100,percent);

chassis.drive_distance(-30,180,8,3,1,100,500);


Matchloader.set(false);

chassis.drive_distance(-30,180,3,3,1,100,2500);


break;


case 11: //


break;


case 12: //


break;

case 13: // 

break;
}
}






/*---------------------------------------------------------------------------*/
/* */
/* User Control Task */
/* */
/* This task is used to control your robot during the user control phase of */
/* a VEX Competition. */
/* */
/* You must modify the code to add your own robot specific commands here. */
/*---------------------------------------------------------------------------*/




void usercontrol(void) {
// User control code here, inside the loop




vex::task driveTask = vex::task( driveFunction );



//vex::task intakeTask = vex::task( intakeFunction );


//vex::task pullerTask = vex::task( pullerFunction );


//vex::task ThrowAwayRedTask = vex::task( ThrowAwayRedFunction );


//vex::task rumbleTask = vex::task( rumbleFunction ); //this function is just for testing tasks


//vex::task ThrowAwayBlueTask = vex::task( ThrowAwayBlueFunction );


//vex::task WallStakesTask = vex::task( WallStakesFunction );


//vex::task doinkerTask = vex::task( doinkerFunction );


//vex::task mogoTask = vex::task( mogoFunction );


//vex::task selectorTask = vex::task( selectorFunction );
//vex::task toggle = vex::task( toggleIntakeFunction );
//vex::task match = vex::task( selectorFunction );




while (1) {



//all switched to tasks
toggleIntakeFunction();

//matchloaderFunction();

//chassis.control_arcade();
//driveFunction();
//arcadeDrive();
//chassis.control_tank();
//intakeCode();
//positionCode();
//WallStakesCode();
//selectorCode();
//descoreCode();
//ThrowAwayRed();
//ThrowAwayBlue();
//DoinkerCode();
// MogoCode();
//TempCode();
//arcadeDrive();
//chassis.control_arcade();


wait(20, msec); // Sleep the task for a short amount of time to
// prevent wasted resources.
}
}

 
//
// Main will set up the competition functions and callbacks.
//


int main() {

 // Set up callbacks for autonomous and driver control periods.
 colorsorter1.integrationTime(5);
 colorsorter2.integrationTime(5);

 Competition.autonomous(autonomous);
 Competition.drivercontrol(usercontrol);
 pre_auton();


 //ColorSortBlue and ColorSortRed functions need to be switched before every match

 //ColorSortBlue();
 //tasksToggle();

 //ColorSortRed();

 //antiJamCode(); //now included in colorsortfunctions

 // Run the pre-autonomous function.

 //ColorSortRed();
 // Prevent main from exiting with an infinite loop.
 while (true) {
 wait(100, msec);
 }
}



