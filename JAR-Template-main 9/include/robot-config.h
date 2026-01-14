using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor rightFront;
extern motor rightBack;
extern motor rightMid;
extern motor leftFront;
extern motor leftBack;
extern motor leftMid;
extern controller Controller1;
extern motor Intake;
extern inertial Inertial;
extern digital_out MOGO;
extern distance colordistance;
extern motor Upper;
extern digital_out mogo;
extern optical colorsorter1;
extern optical colorsorter2;
extern optical opticalsorter;
extern distance aligner;
extern motor WallStakes;
extern rotation rotationSensor;
extern digital_out DoinkerRight;
extern digital_out DoinkerLeft;
extern digital_out Doinker;
extern digital_out sorter;
extern digital_out Puller;
extern digital_out Tipper;
extern digital_out descore;
extern digital_out bottomDescore;
extern digital_out hardStop;


extern digital_out Matchloader;
extern digital_out indexer;

extern motor firstStage;
extern motor secondstage;

extern motor scorer;
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );