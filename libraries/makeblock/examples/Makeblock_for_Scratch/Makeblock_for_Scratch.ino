/*************************************************************************
* File Name          : Makeblock_for_Scratch.ino
* Author             : Ander
* Updated            : Ander
* Version            : V1.0.4
* Date               : 05/06/2014
* Description        : Makeblock Electronic for  Scratch's Communication
* License            : CC-BY-SA 3.0
* Copyright (C) 2014 Maker Works Technology Co., Ltd. All right reserved.
* http://www.makeblock.cc/
**************************************************************************/
#include "Makeblock.h"
#include <Wire.h>
#include <SoftwareSerial.h>
MeUltrasonicSensor ultrasonic;
MeTemperature temperature;
MeLightSensor light;
MePotentiometer potentiometer;
MeJoystick joystick;
MeGyro gyro;
MeRGBLed led;
MeDCMotor motor;
typedef struct MeModule
{
    int device;
    int port;
    int slot;
    int index;
    float values[3];
} MeModule;

union{
    byte byteVal[4];
    float floatVal;
    long longVal;
}val;

MeModule modules[12];

unsigned char digitalPins[12]={0};
unsigned char analogPins[12]={A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11};
unsigned char digitalModes[12]={0,0,0,0,0,0,0,0,0,0,0,0};
unsigned char analogModes[12]={0,0,0,0,0,0,0,0,0,0,0,0};
void setup(){
 Serial.begin(115200); 
 buzzerOn();
 delay(100);
 buzzerOff();
}
int len = 52;
unsigned char buffer[52];
byte index = 0;
byte dataLen;
byte modulesLen=0;
boolean isStart = false;
//unsigned char test[10]={0xff,0x55,2,0x6,0x8,80,0,50,0,0};
//byte testIndex = 0;
#define VERSION 0
#define ULTRASONIC_SENSOR 1
#define TEMPERATURE_SENSOR 2
#define LIGHT_SENSOR 3
#define POTENTIONMETER 4
#define JOYSTICK 5
#define GYRO 6
#define MOTOR 7
#define RGBLED 8
#define DIGITAL 9
#define ANALOG 10
#define PWM 11

void loop(){
  if(Serial.available()){
    unsigned char c = Serial.read();
//    c = test[testIndex];
//    testIndex++;
//    if(testIndex>10){
//     testIndex=0; 
//    }
    if(c==0x55&&isStart==false){
     if(buffer[0]==0xff){
      index=1; 
      isStart = true;
     }
    }else{
      if(!isStart){
       if(c==0xff){
         index=0;
         buffer[index]=c;
       }
      }else{
        if(index==3){
         dataLen = c; 
        }else if(index>3){
          dataLen--;
        }
        buffer[index]=c;
      }
    }
     index++;
     if(isStart&&dataLen==0&&index>3){
      parseData(); 
      index=0;
     }
  }
  
}
void parseData(){
  isStart = false;
  switch(buffer[2]){
    case 0x1:{
        int l = buffer[3]/2;
        for(int i=0;i<l;i++){
          int device = buffer[i*2+4];
          int port = (buffer[i*2+5]&0xf0)>>4;
          int slot = buffer[i*2+5]&0xf;
          modules[i].device = device;
          modules[i].port = port;
          modules[i].slot = slot;
        }
        modulesLen = l;
        readModules();
      }
     break;
     case 0x3:{
        Serial.write(0xff);
        Serial.write(0x55);
        Serial.write(0x3);
        readSensor(DIGITAL,0,0);
        readSensor(ANALOG,0,0);
        Serial.println();
     }
     break;
     case 0x2:{
        int l = buffer[3];
        int dataIndex = 4;
        while(l>dataIndex-4){
          int device = buffer[dataIndex];
          dataIndex++;
          int port = (buffer[dataIndex]&0xf0)>>4;
          int slot = buffer[dataIndex]&0xf;
          dataIndex++;
          MeModule module;
          module.device = device;
          module.port = port;
          module.slot = slot;
          if(device==RGBLED){
            module.index = buffer[dataIndex++];
            module.values[0]=buffer[dataIndex++];
            module.values[1]=buffer[dataIndex++];
            module.values[2]=buffer[dataIndex++];
            led.reset(module.port);
            digitalModes[led.pin1()-2]=1;
            digitalModes[led.pin2()-2]=1;
            if(module.index>0){
              led.setColorAt(module.index-1,module.values[0],module.values[1],module.values[2]);
            }else{
              for(int t=0;t<led.getNumber();t++){
                led.setColorAt(t,module.values[0],module.values[1],module.values[2]);
              }
            }
            led.show();
            Serial.write(0xff);
            Serial.write(0x55);
            Serial.println();
          }else if(device==MOTOR){
            val.byteVal[0]=buffer[dataIndex++];
            val.byteVal[1]=buffer[dataIndex++];
            val.byteVal[2]=buffer[dataIndex++];
            val.byteVal[3]=buffer[dataIndex++];
            module.values[0]=val.floatVal;
            motor.reset(module.port);
            motor.run(module.values[0]);
            digitalModes[motor.pin1()-2]=1;
            digitalModes[motor.pin2()-2]=1;
            Serial.write(0xff);
            Serial.write(0x55);
            Serial.println();
          }else if(device==DIGITAL){
            if(module.slot==1){
              val.byteVal[0]=buffer[dataIndex++];
              val.byteVal[1]=buffer[dataIndex++];
              val.byteVal[2]=buffer[dataIndex++];
              val.byteVal[3]=buffer[dataIndex++];
              pinMode(module.port,OUTPUT);
              digitalModes[module.port-2]=1;
              analogWrite(module.port,val.floatVal==1?1023:val.floatVal);
            }else{
              pinMode(module.port,INPUT);
              digitalModes[module.port-2]=0;
            }
            Serial.write(0xff);
            Serial.write(0x55);
            Serial.println();
          }else if(device==ANALOG||device==PWM){
            if(module.slot==1){
              val.byteVal[0]=buffer[dataIndex++];
              val.byteVal[1]=buffer[dataIndex++];
              val.byteVal[2]=buffer[dataIndex++];
              val.byteVal[3]=buffer[dataIndex++];
              pinMode(module.port,OUTPUT);
              analogModes[module.port]=1;
              analogWrite(module.port,val.floatVal);
            }else{
              pinMode(module.port,INPUT);
              analogModes[module.port]=0;
            }
            Serial.write(0xff);
            Serial.write(0x55);
            Serial.println();
          }
        }
      }
      break;
  }
}
void readModules(){
    Serial.write(0xff);
    Serial.write(0x55);
    Serial.write(0x1);
    readSensor(0,0,0);
    for(int i=0;i<modulesLen;i++){
      MeModule module = modules[i];
      readSensor(module.device,module.port,module.slot);
    }
    Serial.println();
}
void sendValue(float value){ 
     val.floatVal = value;
     Serial.write(val.byteVal[0]);
     Serial.write(val.byteVal[1]);
     Serial.write(val.byteVal[2]);
     Serial.write(val.byteVal[3]);
}
void readSensor(int device,int port,int slot){
  float value;
  switch(device){
   case  ULTRASONIC_SENSOR:{
     if(ultrasonic.getPort()!=port){
       ultrasonic.reset(port);
     }
     value = ultrasonic.distanceCm();
     sendValue(value);
   }
   break;
   case  TEMPERATURE_SENSOR:{
     if(temperature.getPort()!=port||temperature.getSlot()!=slot){
       temperature.reset(port,slot);
     }
     value = temperature.temperature();
     if(value==-100){
       value = temperature.temperature();
       if(value>-100){
         sendValue(value);
       }
     }
   }
   break;
   case  LIGHT_SENSOR:{
     if(light.getPort()!=port){
       light.reset(port);
     }
     value = light.strength();
     sendValue(value);
   }
   break;
   case  POTENTIONMETER:{
     if(potentiometer.getPort()!=port){
       potentiometer.reset(port);
     }
     value = potentiometer.read();
     sendValue(value);
   }
   break;
   case  JOYSTICK:{
     if(joystick.getPort()!=port){
       joystick.reset(port);
     }
     value = joystick.readX();
     sendValue(value);
     value = joystick.readY();
     sendValue(value);
   }
   break;
   case  GYRO:{
     if(gyro.getPort()!=port){
       gyro.reset(port);
       gyro.begin();
     }
     gyro.update();
     value = gyro.angleX();
     sendValue(value);
     value = gyro.angleY();
     sendValue(value);
     value = gyro.angleZ();
     sendValue(value);
   }
   break;
   case  RGBLED:{
     sendValue(0);
   }
   break;
   case  VERSION:{
     sendValue(1.0506);
   }
   break;
   case  DIGITAL:{
     int result = 0;
     for(int i=0;i<12;i++){
       if(digitalModes[i]==0){
         pinMode(i+2,INPUT);
         result |= digitalRead(i+2)<<(11-i);
       }
     }
     sendValue(result);
   }
   break;
   case  ANALOG:{
     for(int i=0;i<12;i++){
       if(analogModes[i]==0){
         if(i==6||i==7||i==8||i==9||i==10||i==12){
          if(digitalModes[i-2]==1){
            sendValue(0);
           continue;
          } 
         }
         pinMode(analogPins[i],INPUT);
         sendValue(analogRead(analogPins[i]));
       }
     }
   }
   break;
  }
}
