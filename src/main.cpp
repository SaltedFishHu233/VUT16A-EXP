/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// LUB                  motor         1               
// LUF                  motor         5               
// RUB                  motor         6               
// RUF                  motor         10              
// LBB                  motor         11              
// LBF                  motor         15              
// Roller               motor         8               
// Wing                 digital_out   A               
// Intake               digital_out   H               
// RBB                  motor         16              
// RBF                  motor         20              
// Gyro                 inertial      3               
// CataEyes             optical       14              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
//#include "STDLib.cpp"
using namespace vex;

#include "math.h"

struct ChassisDataSet{
  int Left;
  int Right;
  double Avg;
  int Diff;
  int HDG;
};

void Zeroing(bool dist, bool HDG)
{
  if(dist){
  LUF.resetPosition();
  LUB.resetPosition();
  LBF.resetPosition();
  LBB.resetPosition();
  RUF.resetPosition();
  RUB.resetPosition();
  RBF.resetPosition();
  RBB.resetPosition();
  }
  if(HDG){
    Gyro.setHeading(0,degrees);
  }
}

ChassisDataSet ChassisUpdate()
{
  ChassisDataSet CDS;
  CDS.Left=(LUF.position(degrees)+LUB.position(degrees)+LBF.position(degrees)+LBB.position(degrees))/4;
  CDS.Right=(RUF.position(degrees)+RUB.position(degrees)+RBF.position(degrees)+RBB.position(degrees))/4;
  CDS.Avg=(double)(CDS.Left+CDS.Right)/2;
  CDS.Diff=CDS.Left-CDS.Right;
  CDS.HDG=Gyro.heading(degrees);

  return CDS;
}
void BStop()
{
  LUF.setStopping(brake);
  LUB.setStopping(brake);
  LBF.setStopping(brake);
  LBB.setStopping(brake);
  RUF.setStopping(brake);
  RUB.setStopping(brake);
  RBF.setStopping(brake);
  RBB.setStopping(brake);

  LUF.stop();
  LUB.stop();
  LBF.stop();
  LBB.stop();
  RUF.stop();
  RUB.stop();
  RBF.stop();
  RBB.stop();

}

void CStop()
{
  LUF.setStopping(coast);
  LUB.setStopping(coast);
  LBF.setStopping(coast);
  LBB.setStopping(coast);
  RUF.setStopping(coast);
  RUB.setStopping(coast);
  RBF.setStopping(coast);
  RBB.setStopping(coast);

  LUF.stop();
  LUB.stop();
  LBF.stop();
  LBB.stop();
  RUF.stop();
  RUB.stop();
  RBF.stop();
  RBB.stop();
}

void LWingB()
{
  LWing.setStopping(hold);
  LWing.stop();
}

void RWingB()
{
  RWing.setStopping(hold);
  RWing.stop();
}

void Move(int left, int right)
{
LUF.setMaxTorque(100,percent);
LUB.setMaxTorque(100,percent);
LBF.setMaxTorque(100,percent);
LBB.setMaxTorque(100,percent);
RUF.setMaxTorque(100,percent);
RUB.setMaxTorque(100,percent);
RBF.setMaxTorque(100,percent);
RBB.setMaxTorque(100,percent);

LUF.spin(forward,(double)left/100.0*11,volt);
LUB.spin(forward,(double)left/100.0*11,volt);
LBF.spin(forward,(double)left/100.0*11,volt);
LBB.spin(forward,(double)left/100.0*11,volt);
RUF.spin(forward,(double)right/100.0*11,volt);
RUB.spin(forward,(double)right/100.0*11,volt);
RBF.spin(forward,(double)right/100.0*11,volt);
RBB.spin(forward,(double)right/100.0*11,volt);
}

void RunRoller(int val)
{
Roller.setMaxTorque(100,percent);
Roller.spin(forward,(double)val/100.0*11,volt);
}
/*void RunArm(int Pow)
{
LArm.setMaxTorque(100,percent);
RArm.setMaxTorque(100,percent);
LArm.spin(forward,(double)Pow/100.0*11,volt);
RArm.spin(forward,(double)Pow/100.0*11,volt);
}*/

//AutoSect;
//This section includes all auto codes

struct PIDDataSet{
  double kp;
  double ki;
  double kd;
};
int PrevE;//Error at t-1
void MoveEncoderPID(PIDDataSet KVals, int Speed, double dist,double AccT, double ABSHDG,bool brake){
  double CSpeed=0;
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  double LGV=0;//define local gyro variable.
  PrevE=0;
  double Correction=0;
  Brain.Screen.clearScreen();

  while(fabs(SensorVals.Avg) <= fabs(dist))
  {
if(fabs(CSpeed)<fabs((double)Speed))
{
  CSpeed+=Speed/AccT*0.02;
}

  SensorVals=ChassisUpdate();
  LGV=SensorVals.HDG-ABSHDG;
  if(LGV>180) LGV=LGV-360;
  PVal=KVals.kp*LGV;
  IVal=IVal+KVals.ki*LGV*0.02;
  DVal=KVals.kd*(LGV-PrevE);

  Correction=PVal+IVal+DVal/0.02;

  Move(-CSpeed+Correction,-CSpeed-Correction);
  PrevE=LGV;
  wait(20, msec);
  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}

void TurnMaxTimePID(PIDDataSet KVals,double DeltaAngle,double TE, bool brake){
  double CSpeed=0;
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  double LGV=0;
  PrevE=0;
  double Correction=0;
  Brain.Timer.reset();

  while(Brain.Timer.value() <= TE)
  {
  SensorVals=ChassisUpdate();
  LGV=SensorVals.HDG-DeltaAngle;
  if(LGV>180) LGV=LGV-360;
  PVal=KVals.kp*LGV;
  IVal=IVal+KVals.ki*LGV*0.02;
  DVal=KVals.kd*(LGV-PrevE);

  Correction=PVal+IVal+DVal/0.02;

  Move(-CSpeed+Correction,-CSpeed-Correction);
  PrevE=LGV;
  wait(20, msec);
  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}

void TurnMaxTimePIDWOneSide(PIDDataSet KVals,double DeltaAngle,double TE, bool brake){
  double CSpeed=0;
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  double LGV=0;
  PrevE=0;
  double Correction=0;
  double LV,RV;
  Brain.Timer.reset();

  while(Brain.Timer.value() <= TE)
  {
  SensorVals=ChassisUpdate();
  LGV=SensorVals.HDG-DeltaAngle;
  if(LGV>180) LGV=LGV-360;
  PVal=KVals.kp*LGV;
  IVal=IVal+KVals.ki*LGV*0.02;
  DVal=KVals.kd*(LGV-PrevE);

  Correction=PVal+IVal+DVal/0.02;
LV=-CSpeed+Correction;
RV=-CSpeed-Correction;
if(LV>=0)LV=0;
if(RV>=0)RV=0;
  Move(LV,RV);
  PrevE=LGV;
  wait(20, msec);
  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}


void MoveTimePID(PIDDataSet KVals, int Speed, double TE,double AccT,double ABSHDG, bool brake){
  double CSpeed=0;
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  double LGV=0;
  PrevE=0;
  double Correction=0;
  Brain.Timer.reset();

  while(Brain.Timer.value() <= TE)
  {
if(fabs(CSpeed)<fabs((double)Speed))
{
  CSpeed+=Speed/AccT*0.02;
}

  SensorVals=ChassisUpdate();
    LGV=SensorVals.HDG-ABSHDG;
  if(LGV>180) LGV=LGV-360;
  PVal=KVals.kp*LGV;
  IVal=IVal+KVals.ki*LGV*0.02;
  DVal=KVals.kd*(LGV-PrevE);

  Correction=PVal+IVal+DVal/0.02;

  Move(-CSpeed+Correction,-CSpeed-Correction);
  PrevE=LGV;
  wait(20, msec);
  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}


// A global instance of competition
competition Competition;

int JB;
int PB;
int PX;
int JX;


// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  PX=0;
  JX=0;
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
Gyro.calibrate();

//Ensure Robot Launch Position is set before auto proceeds, once plugged into field control,
//start program and do not temper bot under all circumstances

//1. IF ANY ADJUSTMENT IS NEEDED, QUIT PROGRAM, THEN ADJUST, RESTART PROGRAM AFTER ADJUSTMENTS COMPLETED
//2. DO NOT START PROGRAM BEFORE PLUGGING IN FIELD CONTROL, THIS MAY DISABLE AUTO
//3. ONLY SIGNAL JUDGES TO BEGIN MATCH AFTER THE ZEROING PROMPT ON SCREEN HAS CLEARED

//Print precautionary message
//Brain.Screen.drawRectangle(0,0,500,500);

Brain.Screen.setFont(monoXL);
Brain.Screen.setPenColor("#39FF14");
Brain.Screen.setCursor(2,10);
Brain.Screen.print("FLIR TIMEOUT");


  waitUntil(!Gyro.isCalibrating());


Zeroing(true,true);

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  

  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
CStop();
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}
int RV;
int LV;
int DriveTask(void){
  while(true)
  {
    RV=Controller1.Axis3.position(percent)-Controller1.Axis1.position(percent);
    LV=Controller1.Axis3.position(percent)+Controller1.Axis1.position(percent);
    //if(PX==1)PTOMove(LV,RV);
    Move(LV,RV);
  }

return 0;
}

int ATask(void)
{
  //double pow;
    while(true)
  {
/*    pow=((Controller1.ButtonL2.pressing()-Controller1.ButtonL1.pressing())*100);
    RunRoller(pow);
    if(PB==1)RunArm((Controller1.ButtonUp.pressing()-Controller1.ButtonDown.pressing())*100);
  RunPuncher((Controller1.ButtonB.pressing())*100);*/
  RunRoller((Controller1.ButtonR2.pressing()-Controller1.ButtonR1.pressing())*100);
    //RunArm((Controller1.ButtonL1.pressing()-Controller1.ButtonL2.pressing())*100);
  
          if(PX==0&&Controller1.ButtonL1.pressing()&&JX==0)
    {
      JX=1;
      PX=1;
    }
    else if(!Controller1.ButtonL1.pressing())JX=0;
    else if(PX==1&&Controller1.ButtonL1.pressing()&&JX==0)
    {
      JX=1;
      PX=0;
    }
    if(PX==1){
      if(LWing.position(degrees)<90)LWing.spin(forward,11,volt);
      else LWingB();

      if(RWing.position(degrees)<90)RWing.spin(forward,11,volt);
      else RWingB();
    }
    else{
      if(LWing.position(degrees)>10)LWing.spin(reverse,11,volt);
      else LWingB();
      if(RWing.position(degrees)>10)RWing.spin(reverse,11,volt);
      else RWingB();
    }
    
  }
  return 0;
}
int JY;
int PY;
int PTask(void)
{
    while(true)
  {
    /*if(PY==0&&Controller1.ButtonY.pressing()&&JY==0)
    {
      JY=1;
      PY=1;
      Intake.set(true);
    }

    else if(!Controller1.ButtonY.pressing())JY=0;

    else if(PY==1&&Controller1.ButtonY.pressing()&&JY==0)
    {
      JY=1;
      PY=0;
      Intake.set(false);
    }*/




        if(PB==0&&Controller1.ButtonX.pressing()&&JB==0)
    {
      JB=1;
      PB=1;
      Wing.set(true);
    }

    else if(!Controller1.ButtonX.pressing())JB=0;

    else if(PB==1&&Controller1.ButtonX.pressing()&&JB==0)
    {
      JB=1;
      PB=0;
      Wing.set(false);
    }
    
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  Intake.set(false);
  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    task Dtask=task(DriveTask);
    task Atask=task(ATask);
    task Ptask=task(PTask);
    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
