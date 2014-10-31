/*************************************************************************
 * File Name          : PortControl.ino
 * Author             : Ander
 * Updated            : Ander
 * Version            : V1.0.0
 * Date               : 03/03/2014
 * Description        : using bluetooth with mobile app to communicate the 
                        sensor and control motors.
 * License            : CC-BY-SA 3.0
 * Copyright (C) 2014 Maker Works Technology Co., Ltd. All right reserved.
 * http://www.makeblock.cc/
 **************************************************************************/
#include "Makeblock.h"
#include <SoftwareSerial.h>
#include <Wire.h>
MeSerial serial(PORT_4);
MeDCMotor motor;
MeServo servo;
MeRGBLed led;
MeUltrasonicSensor ultrasonic;
MeLineFinder linefinder;
MeLimitSwitch limitswitch;
MeJoystick joystick;
MeSoundSensor soundsensor;
MeLightSensor lightsensor;
Me4Button button;
MeTemperature temperature;
char *device;
char *method;
int port;
char *pin;
int value;
int slot;
int ultrasonic_port=-1;
int switch_port=-1;
int switch_slot=1;
long servoTime = 0;
bool isServoRun = false;
char *color;
float s1 = 0;
float s2 = 0;
typedef struct{
  int type;
  int mode;
  int slot[2];
}Module;
Module ports[10];
Module pins[18];
int analogPins[6] = {A0,A1,A2,A3,A4,A5};
void addDevice(){
  if(port>0){
    int index = port-1;
    if(strcmp(device,"motor")==0){
      ports[index].type=1;
    }else if(strcmp(device,"servo")==0){
      ports[index].type=2;
      ports[index].slot[slot-1]=1;
    }else if(strcmp(device,"Ultrasonic")==0){
      ports[index].type=3;
    }else if(strcmp(device,"LineFinder")==0){
      ports[index].type=4;
    }else if(strcmp(device,"LimitSwitch")==0){
      ports[index].type=5;
      ports[index].slot[slot-1]=1;
    }else if(strcmp(device,"LightSensor")==0){
      ports[index].type=6;
    }else if(strcmp(device,"SoundSensor")==0){
      ports[index].type=7;
    }else if(strcmp(device,"Temperature")==0){
      ports[index].type=8;
      ports[index].slot[slot-1]=1;
    }else if(strcmp(device,"Joystick")==0){
      ports[index].type=9;
    }else if(strcmp(device,"Button")==0){
      ports[index].type=10;
    }
  }
}
void runDevice(){
  
  if(port>0){
    int index = port-1;
    if(strcmp(device,"motor")==0){
      motor.reset(port);
      motor.run(value);
    }else if(strcmp(device,"servo")==0){
      servo.reset(port,1-slot);
      servo.begin();
      servo.write(value);
      isServoRun = true;
      servoTime = millis();
    }else if(strcmp(device,"RGBLed")==0){
     if(ports[index].type!=11){
        ports[index].type=11;
        led.reset(port); 
     }
     color = serial.getParamCode("c");
      uint8_t r = toHEX(color[0])*16+toHEX(color[1]);
      uint8_t g = toHEX(color[2])*16+toHEX(color[3]);
      uint8_t b = toHEX(color[4])*16+toHEX(color[5]);
      if(r==0&&g==0&&b==0){
        return;
      }
      int i;
      for(i=0;i<led.getNumber();i++){
        led.setColorAt(i,r,g,b);
      }
      led.sync();
    } 
  }
}
int toHEX(char c){
  if(c>=48&&c<=57){
    return c-48;
  }
  if(c>=97&&c<=102){
    return c-87;
  }
  return 0;
}
void checkDevice(){
  int i,portNum;
  for(i=0;i<10;i++){
    portNum = i+1;
    if(ports[i].type>0){
      switch(ports[i].type){
        case 3:{
          ultrasonic.reset(portNum);
          int dist = ultrasonic.distanceCm();
          sendCommand("Ultrasonic/P",portNum,-1,dist);
        }
        break;
        case 4:{
          linefinder.reset(portNum);
          int state = linefinder.readSensors();
          sendCommand("LineFinder/P",portNum,-1,state);
        }
        break;
        case 5:{
          limitswitch.reset(portNum,ports[i].slot[0]==1?1:2);
          int touched = limitswitch.touched();
          sendCommand("LimitSwitch/P",portNum,ports[i].slot[0]==1?1:2,touched==0,true);
        }
        break;
        case 6:{
          lightsensor.reset(portNum);
          int light = lightsensor.strength();
          sendCommand("LightSensor/P",portNum,-1,light);
        }
        break;
        case 7:{
          soundsensor.reset(portNum);
          int sound = soundsensor.strength();
          sendCommand("SoundSensor/P",portNum,-1,sound);
        }
        break;
        case 8:{
          temperature.reset(portNum,ports[i].slot[0]==1?1:2);
          float t = temperature.temperature();
          if(t==-100){
            t = temperature.temperature();
          }
          if(t>-100){
            sendCommand("Temperature/P",portNum,ports[i].slot[0]==1?1:2,t);
          }
        }
        break;
        case 9:{
          joystick.reset(portNum);
          int dist = joystick.readX();
          serial.print("Joystick/P");
          serial.print(portNum);
          serial.print(ports[i].slot[0]==1?"/X-Axis":"/Y-Axis");
          serial.print(" ");
          serial.println(dist);
        }
        break;
        case 10:{
          button.reset(portNum);
          int state = button.pressed();
          sendCommand("Button/P",portNum,-1,state);
        }
        break;
      }
    }
  }
  for(i=0;i<18;i++){
    if(pins[i].type==1){
      sendCommand("digital/D",i+2,-1,digitalRead(i+2));
    }else if(pins[i].type==2){
      sendCommand("analog/A",i-12,-1,analogRead(analogPins[i-12]));
    }
  }
  if(isServoRun&&millis()-servoTime>500){
     isServoRun = false;
     servo.detach(); 
  }
}
void sendCommand(const char*cmd,int cPort,int cSlot,double v){
  sendCommand(cmd,cPort,cSlot,v,false);
}
void sendCommand(const char*cmd,int cPort,int cSlot,double v,bool isBool){
  Serial.print(cmd);
  Serial.print(":");
  Serial.println(v);
  serial.print(cmd);
  serial.print(cPort);
  if(cSlot>-1){
    serial.print("/Slot");
    serial.print(cSlot);
  }
  serial.print(" ");
  if(isBool){
    serial.println(v!=0?"true":"false");
  }else{
    serial.println(v);
  }
}
void setPinMode(){
  if(value==0){
    return;
  }
  if(pin[0]=='D'){
    pin[0]='0';
    int p = atoi(pin);
    pinMode(p,value==2?OUTPUT:INPUT);
    if(value==1){
      pins[p-2].type = 1;
    }else{
      pins[p-2].type = 0;
    }
  }else if(pin[0]=='A'){
    pin[0]='0';
    int p = atoi(pin);
    pinMode(analogPins[p],value==2?OUTPUT:INPUT);
    if(value==1){
      pins[p+12].type = 2;
    }else{
      pins[p+12].type = 0;
    }
  }
}
void pinWrite(){
  if(pin[0]=='D'){
    pin[0]='0';
    digitalWrite(atoi(pin),value==2?HIGH:LOW);
  }else if(pin[0]=='A'){
    pin[0]='0';
    digitalWrite(analogPins[atoi(pin)],value==2?HIGH:LOW);
  }
}

long currentTime = 0;
long sampleTime = 100;
long baudrate = 115200;

void setup() {
  serial.begin(baudrate);
  serial.println("Application Start");
  Serial.begin(9600);
  pinMode(10,OUTPUT);
  analogWrite(10,0);
}
void loop() {
//  if(serial.available()){
//    char c = serial.read();
//    Serial.print(c);
//  }
//  return;
  long time = millis()-currentTime;
  if(time>sampleTime||time<0){
    currentTime = millis();
    checkDevice();
  }
  
  if(serial.paramAvailable()){
    device = serial.getParamCode("device");
    port = serial.getParamValue("port");
    pin = serial.getParamCode("pin");
    value = serial.getParamValue("value");
    slot = serial.getParamValue("slot");
    method = serial.getParamCode("method");
    s1 = serial.getParamValue("s1");
    s2 = serial.getParamValue("s2");
    if(s1!=0||s2!=0){
      
      motor.reset(M1);
      motor.run(-s1);
      motor.reset(M2);
      motor.run(-s2);
      return;
    }
    int p = serial.getParamValue("p");
    if(p>0){
     port = p;
     device = "RGBLed";
     runDevice(); 
     return; 
    }
    if(strcmp(device,"reset")==0){
      int i;
      if(port==0){
        for(i=0;i<10;i++){
           ports[i].type = 0;
           ports[i].slot[0] = 0;
           ports[i].slot[1] = 0;
        }
        for(i=0;i<18;i++){
           pins[i].type = 0;
           pins[i].slot[0] = 0;
           pins[i].slot[1] = 0;
        }
        motor.stop();
      }else{
        ports[port-1].type = 0;
        if(port>8){
          motor.stop();
        }
      }
      return;
    }
    if(strcmp(method,"add")==0){
     addDevice(); 
     return;
    }else if(strcmp(method,"run")==0){
     runDevice(); 
     return;
    }else if(strcmp(method,"mode")==0){
     setPinMode(); 
     return;
    }else if(strcmp(method,"write")==0){
     pinWrite(); 
     return;
    }else{
     method =  serial.getParamCode("m");
     device = serial.getParamCode("d");
     if(strcmp(method,"run")==0){
       runDevice();
     }
    }
  }
}
