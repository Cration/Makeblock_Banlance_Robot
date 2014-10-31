/*************************************************************************
* File Name          : CarControl.ino
* Author             : Jasen
* Updated            : Xiaoyu
* Version            : V1.0.0
* Date               : 3/3/2014
* Description        : Demo code for Makeblock Starter Robot kit,two motors
                       connect on the M1 and M2 port of baseshield or baseboard, The Bluetooth module
                       connect on port 4 and the Ultrasonic sensor connect on port 3.
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include "Makeblock.h"
#include <SoftwareSerial.h>
#include <Wire.h>

MeDCMotor MotorL(M1);  
MeDCMotor MotorR(M2);
MeBluetooth serial(PORT_4);
MeUltrasonicSensor UltrasonicSensor(PORT_3);
int moveSpeed = 190;
int turnSpeed = 200;
int distance=0;
int randnum = 0;
boolean leftflag,rightflag;
int minSpeed = 48;
int factor = 23;

void setup()
{
    leftflag=false;
    rightflag=false;
    serial.begin(115200);
}
int s1=0;
int s2=0;
int s3=0;

void loop()
{
  if(serial.paramAvailable()){
    s1 = serial.getParamValue("s1");
    s2 = serial.getParamValue("s2");
    if(serial.getParamValue("s3")>0){
      s3 = serial.getParamValue("s3");
    }
    MotorL.run(s1);
    MotorR.run(s2);
    return;
  }
   if(s3==2&&millis()%50 == 0)
   {
     Serial.println("ultrasonic");
      ultrCarProcess();
   }
}

void Forward()
{
  MotorL.run(moveSpeed);
  MotorR.run(moveSpeed);
}
void Backward()
{
  MotorL.run(-moveSpeed);
  MotorR.run(-moveSpeed);
}

void BackwardAndTurnLeft()
{
  MotorL.run(-moveSpeed/2);
  MotorR.run(-moveSpeed);
}
void BackwardAndTurnRight()
{
  MotorL.run(-moveSpeed);
  MotorR.run(-moveSpeed/2);
}
void TurnLeft()
{
  MotorL.run(-moveSpeed);
  MotorR.run(moveSpeed);
}
void TurnRight()
{
  MotorL.run(moveSpeed);
  MotorR.run(-moveSpeed);
}
void Stop()
{
  MotorL.run(0);
  MotorR.run(0);
}
void ChangeSpeed(int spd)
{
  moveSpeed = spd;
}

void ultrCarProcess()
{
  distance = UltrasonicSensor.distanceCm();
  randomSeed(analogRead(A4));
  if(distance>10&&distance<40)
  {
    randnum=random(300);
    if(randnum > 190 && !rightflag)
    {
      leftflag=true;
      TurnLeft();   
    }
    else
    {
      rightflag=true;
      TurnRight();  
    }
  }
  else if(distance<10)
  {
     randnum=random(300);
    if(randnum > 190)BackwardAndTurnLeft();
    else BackwardAndTurnRight();
  }
  else
  {
      leftflag=false;
      rightflag=false;
      Forward();
  }
}
