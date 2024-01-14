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
/*
void Zeroing(bool dist, bool HDG)
{
  if(dist){
  LF.resetPosition();
  LM.resetPosition();
  LB.resetPosition();
  RF.resetPosition();
  RM.resetPosition();
  RB.resetPosition();
  }
  if(HDG){
    Gyro.setHeading(0,degrees);
  }
}

ChassisDataSet ChassisUpdate()
{
  ChassisDataSet CDS;
  CDS.Left=(LF.position(degrees)+LM.position(degrees)+LB.position(degrees))/3;
  CDS.Right=(RF.position(degrees)+RM.position(degrees)+RB.position(degrees))/3;
  CDS.Avg=(double)(CDS.Left+CDS.Right)/2;
  CDS.Diff=CDS.Left-CDS.Right;
  if(Gyro.heading(degrees)>180) CDS.HDG=Gyro.heading(degrees)-360;
  else CDS.HDG=Gyro.heading(degrees);

  return CDS;
}

void BStop()
{
LF.setStopping(hold);
LM.setStopping(hold);
LB.setStopping(hold);
RF.setStopping(hold);
RM.setStopping(hold);
RB.setStopping(hold);

LF.stop();
LM.stop();
LB.stop();
RF.stop();
RM.stop();
RB.stop();
}

void CStop()
{
LF.setStopping(coast);
LM.setStopping(coast);
LB.setStopping(coast);
RF.setStopping(coast);
RM.setStopping(coast);
RB.setStopping(coast);

LF.stop();
LM.stop();
LB.stop();
RF.stop();
RM.stop();
RB.stop();
}

*/


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


/*void PTOMove(int left, int right)
{
LF.setMaxTorque(100,percent);
//LM.setMaxTorque(100,percent);
LB.setMaxTorque(100,percent);
RF.setMaxTorque(100,percent);
//RM.setMaxTorque(100,percent);
RB.setMaxTorque(100,percent);

LF.spin(forward,(double)left/100.0*11,volt);
//LM.spin(forward,(double)left/100.0*11,volt);
LB.spin(forward,(double)left/100.0*11,volt);
RF.spin(forward,(double)right/100.0*11,volt);
//RM.spin(forward,(double)right/100.0*11,volt);
RB.spin(forward,(double)right/100.0*11,volt);
}*/

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
/*
void RunCata(int Pow)
{
Cata.setMaxTorque(100,percent);
Cata.spin(forward,(double)Pow/100.0*11,volt);
}
void BCata()
{
Cata.setStopping(hold);
Cata.stop();
}*/


//AutoSect;
//This section includes all auto codes
/*
struct PIDDataSet{
  double kp;
  double ki;
  double kd;
};
int PrevE;//Error at t-1
void MoveEncoderPID(PIDDataSet KVals, int Speed, double dist, bool brake){
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  PrevE=0;
  double Correction;

  while(fabs(SensorVals.Avg) <= fabs(dist))
  {
    if(fabs(dist)-fabs(SensorVals.Avg)<100){
      if(Speed<0)Speed=-25;
      else Speed=25;
    }
  SensorVals=ChassisUpdate();
  PVal=KVals.kp*SensorVals.Diff;
  IVal=IVal+KVals.ki*SensorVals.Diff*0.02;
  DVal=KVals.kd*(SensorVals.Diff-PrevE)/0.02;

  Correction=PVal+IVal+DVal;

  Move(-Speed-Correction,-Speed+Correction);
  PrevE=SensorVals.Diff;
  wait(20, msec);

  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}

void MoveEncoderPIDWPreLoad(PIDDataSet KVals, int Speed, double dist, bool brake){
  Zeroing(true,false);
  ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
  double PVal=0;
  double IVal=0;
  double DVal=0;
  PrevE=0;
  double Correction;

  while(fabs(SensorVals.Avg) <= fabs(dist))
  {
    if(fabs(dist)-fabs(SensorVals.Avg)<100){
      if(Speed<0)Speed=-25;
      else Speed=25;
    }
    if(!CataEye.isNearObject()) RunCata(100);
    else BCata();
  SensorVals=ChassisUpdate();
  PVal=KVals.kp*SensorVals.Diff;
  IVal=IVal+KVals.ki*SensorVals.Diff*0.02;
  DVal=KVals.kd*(SensorVals.Diff-PrevE)/0.02;

  Correction=PVal+IVal+DVal;

  Move(-Speed-Correction,-Speed+Correction);
  PrevE=SensorVals.Diff;
  wait(20, msec);

  }
  if(brake){BStop();
  wait(200,msec);}
  else CStop();
}

void CTurn(int left, int right, double dist)
{
  Zeroing(true,true);
    ChassisDataSet SensorVals;
  SensorVals=ChassisUpdate();
 while(fabs((double)SensorVals.Diff) <= fabs(dist))
  {
      SensorVals=ChassisUpdate();
      Move(left,right);
  }
  BStop();
  wait(200,msec);
}

*/
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
  /*
//  Wing.set(true);
//  wait(500,msec);
PIDDataSet TestPara={2.5,0.0001,0.01};
RunArm(0);
Move(-25,-25);
wait(775,msec);
CTurn(50,-50,125);
RunArm(100);
  wait(400,msec);
        //  MoveEncoderPID(TestPara, -50, 200,true);
  for(int i=0;i<25;i++){
BStop();
while(MLB.position(degrees)<150)
{
  MLB.spin(fwd,11,volt);
}

while(MLB.position(degrees)>45)
{
  MLB.spin(reverse,11,volt);
}
  MLB.spin(reverse,0,volt);

wait(750,msec);


    RunArm(100);
  wait(300,msec);
  RunArm(0);
  MoveEncoderPIDWPreLoad(TestPara, -25, 250,false);
  BCata();
    RunRoller(100);
  Move(25,25);
  wait(500,msec);
  Move(0,0);
    wait(200,msec);
  RunArm(-100);
  wait(200,msec);
  RunRoller(0);
    RunArm(0);

  MoveEncoderPID(TestPara, 100, 300,true);
  CTurn(50,-50,40);
RunCata(100);
  wait(400,msec);
    CTurn(-50,50,43);

RunCata(0);
  }
Move(-25,-25);
wait(100,msec);
CTurn(-50,0,140);
while(MLB.position(degrees)<90)
{
  MLB.spin(fwd,11,volt);
}
MLB.setStopping(hold);
MLB.stop();
ChassisDataSet MainSensorVals;
Zeroing(true,true);
MainSensorVals=ChassisUpdate();
while(fabs(MainSensorVals.Avg)<1250){
  MainSensorVals=ChassisUpdate();
Move(-52,-50);}

BStop();
wait(400,msec);


CTurn(-50,0,300);
while(MLB.position(degrees)>30)
{
  MLB.spin(reverse,11,volt);
}
MLB.setStopping(hold);
MLB.stop();

Move(40,40);
wait(2000,msec);

Move(-100,-100);
wait(500,msec);

BStop();
wait(750,msec);

Move(50,50);
wait(1000,msec);

BStop();
wait(200,msec);
CTurn(50,-50,150);
while(MLB.position(degrees)<90)
{
  MLB.spin(fwd,11,volt);
}
MLB.setStopping(hold);
MLB.stop();
Move(-50,-50);
wait(700,msec);
BStop();
CTurn(-100,0,140);
for(int i=0; i<=2;i++)
{
Move(-100,-100);
wait(500,msec);
Move(25,25);
wait(500,msec);

}
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
CStop();*/
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
    /*RunArm((Controller1.ButtonL1.pressing()-Controller1.ButtonL2.pressing())*100);
  
          if(PX==0&&Controller1.ButtonA.pressing()&&JX==0)
    {
      JX=1;
      PX=1;
    }
    else if(!Controller1.ButtonA.pressing())JX=0;
    else if(PX==1&&Controller1.ButtonA.pressing()&&JX==0)
    {
      JX=1;
      PX=0;
    }
  if(PX==1)
  {
      if(Controller1.ButtonB.pressing()) RunCata(100);
      else if(!CataEye.isNearObject()) RunCata(100);
      else{BCata();}
  }
  else
  { 
    if(Controller1.ButtonUp.pressing()) RunCata(-50);
    else RunCata(0);
  }*/
  
    
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
