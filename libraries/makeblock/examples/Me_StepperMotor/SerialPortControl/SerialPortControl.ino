/*************************************************************************
* File Name          : SerialPortControl.ino
* Author             : Evan
* Updated            : Evan
* Version            : V0.3.1
* Date               : 10/09/2013
* Description        : Test for Makeblock Electronic modules of Me_Stepper 
                       Driver. The module can only be connected to the 
                       PORT_1, PORT_2 of Me - Base Shield.. 
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include "Makeblock.h"
#include <SoftwareSerial.h>
#include <Wire.h>

MeStepperMotor stepper(PORT_1,0x05);

void setup()
{
  Serial.begin(9600); 
  stepper.begin(); // initialize stepper driver.
  stepper.setMicroStep(STP_QUARTER); // set micro step. STP_FULL, STP_HALF, STP_QUARTER, STP_EIGHTH, STP_SIXTEENTH
  stepper.setMaxSpeed(5000);         // set max speed. 1-10000
  stepper.setAcceleration(120000);   // set acceleration. 1-200000
  stepper.run(); // output pulse
}

void loop()
{
  if (Serial.available())
    stepper.moveTo(Serial.read()*10);

}

