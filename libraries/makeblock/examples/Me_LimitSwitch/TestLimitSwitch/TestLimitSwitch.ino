/*************************************************************************
* File Name          : TestLimitSwitch.ino
* Author             : Evan
* Updated            : Ander
* Version            : V1.0.1
* Date               : 2/21/2014
* Description        : Example for Makeblock Electronic modules of 
                       Me-LimitSwitch. The module can only be connected to 
                       the PORT_3, PORT_4, PORT_5, and PORT_6 of Me - 
                       Base Shield.
* License            : CC-BY-SA 3.0
* Copyright (C) 2013 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include <Makeblock.h>
#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>

MeLimitSwitch limitSwitch(PORT_6); //Me_LimitSwitch module can only be connected to PORT_3, PORT_4, PORT_5, PORT_6 of base shield or from PORT_3 to PORT_8 of baseboard.

void setup()
{
    Serial.begin(9600);
    Serial.println("Start.");
}
void loop()
{
   if(limitSwitch.touched()) //If the limit switch is up, the "readUpPin" return value is true.
   {
     Serial.println("State: UP.");
     delay(1);
     while(limitSwitch.touched()); //Repeat check the switch state, until released.
     delay(2);
   }
   if(!limitSwitch.touched()){
     Serial.println("State: DOWN.");
     delay(1);
     while(!limitSwitch.touched());
     delay(2);
   }
}