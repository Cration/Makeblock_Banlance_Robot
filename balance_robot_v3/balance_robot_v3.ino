#include <Arduino.h>
#include <Makeblock.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <EEPROM.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#define PI 3.1415926

/******* system params *******/
int debug=0,en=0;
//MeEncoderMotor motor(9);
MeDCMotor m1(M1);
MeDCMotor m2(M2);
MPU6050 accelgyro(0x68);
MePort dbgport(PORT_3);
MeInfraredReceiver ira(PORT_5); 
float  dt=0.01; // 10ms
struct{
  int ver;
  double p, i, d;
  int output_min, output_max;
  int accTime;
  double turn; // the turn speed
  double accAngle;
  double posScaler;
  double speedScaler;
  double sample_time; // in milli second
  double balanceAngle;
  double falling;
  double deadL,deadR; // dead-band config
}pa;
float PTerm,DTerm,ITerm,STerm;
float userAcc;
int userAccCnt;
float userTurn;
int userTurnCnt;
/******* mpu parameters *******/
float Angle; // filtered angle
float setAngle,setAngleLast;
float angleXY;
float gyroZ;
float speed; // the speed from encoder, extra speed compansate
float position,targetPosition; // the position from encoder, the integral term
int16_t acc[3];
int16_t gry[3];
int16_t acc0,gry0;
/******* motor parameters *******/
float pwm; // output pwm from pids loop
float pwmL,pwmR;
float posL,posR;
float posL0,posR0; // last position of encoder

/******* loop delay function *******/
#define FIXDELAY 10*1000 // 10ms loop delay
long time,deltaTime;
void fixdelay()
{
  deltaTime = micros()-time;
  delayMicroseconds(FIXDELAY-deltaTime);
  time = micros();
}

/******* pid calculation ******/
float inputLast,errorLast,errorBeforeLast; // the last error and integral error
float outputInte;
float dout;

float resetPid()
{
  outputInte=0;
  errorLast = 0;
  errorBeforeLast = 0;
  inputLast = pa.balanceAngle;
  ITerm = 0;
  setAngleLast = pa.balanceAngle;
}

float calcPid(float input, float setpoint)
{
  float error;
  error = input - setpoint;
  PTerm = error*pa.p;
  ITerm+=pa.i*error*dt;
  ITerm = constrain(ITerm,pa.output_min,pa.output_max);
  DTerm = pa.d*(input-inputLast)/dt; // not interferenced by setpoint change
  //DTerm = pa.d*gyroZ/dt;  // gyroZ instead of diff error, it is more smooth
  inputLast = input;
  return PTerm+ITerm+DTerm;
}

// incremental pid
float lastSetpoint;
float calcPidIncr(float input, float setpoint)
{
  float error,derror;
  setpoint=constrain(setpoint+userAcc,lastSetpoint-0.05,lastSetpoint+0.05);
  lastSetpoint = setpoint;
  error = input - setpoint;
  derror = error - errorLast;
  PTerm = pa.p*derror;
  ITerm = pa.i*error*dt; // not integral
  ITerm = constrain(ITerm,pa.output_min,pa.output_max);

  DTerm = pa.d*(error-2*errorLast+errorBeforeLast)/dt; // not interferenced by setpoint change
  errorBeforeLast = errorLast;
  errorLast = error;
  dout = PTerm+ITerm+DTerm;
  outputInte+=dout;
  return outputInte;
}

/******* port debug for oscilator *******/
void dbgPortSwitch()
{
  static int a;
  if(a){
    a = 0;
    dbgport.Dwrite1(0);
  }else{
    a = 1;
    dbgport.Dwrite1(1);
  }
}

/******* serial debug command *******/
void parseCommand(char * c)
{
  // all wr command in Upper char
  if(c[0]=='A'){ // set pid params
    char * tmp;
    char * str;
    strtok_r(c, ":", &tmp);
    str = strtok_r(0, ",", &tmp);pa.p=atof(str);
    str = strtok_r(0, ",", &tmp);pa.i=atof(str);
    str = strtok_r(0, ",", &tmp);pa.d=atof(str);
    Serial.print("ack:");Serial.println(c[0]);
  }else if(c[0]=='a'){
    Serial.print("pid:");
    Serial.print(pa.p);Serial.print(',');
    Serial.print(pa.i);Serial.print(',');
    Serial.println(pa.d);
  }else if(c[0]=='B'){ // set motor output min max
    sscanf(c,"B:%d,%d\n",&pa.output_min,&pa.output_max);
    Serial.print("ack:");Serial.println(c[0]);
  }else if(c[0]=='b'){
    Serial.print("motor:");
    Serial.print(pa.output_min);Serial.print(',');
    Serial.println(pa.output_max);
  }else if(c[0]=='C'){ // set sample time, in millis seconds
    int time,accTime;
    sscanf(c,"C:%d,%d\n",&time,&accTime);
    pa.sample_time = time;
    pa.accTime = accTime;
    Serial.print("ack:");Serial.println(c[0]);
  }else if(c[0]=='c'){
    Serial.print("time:");    Serial.print(pa.sample_time);Serial.print(',');Serial.println(pa.accTime);
  }else if(c[0]=='D'){ // enable/disable debug mode
    sscanf(c,"D:%d,%d\n",&debug,&en);
    if(en){
      resetEncoder();
      resetPid();
    }
    Serial.print("ack:");Serial.println(c[0]);
  }else if(c[0]=='d'){
    Serial.print("en:");Serial.print(debug);Serial.print(',');Serial.println(en);
  }else if(c[0]=='E'){ // save to eeprom
    
  }else if(c[0]=='F'){  // angle
    char * tmp;
    char * str;
    strtok_r(c, ":", &tmp);
    str = strtok_r(0, ",", &tmp);pa.falling=atof(str);
    str = strtok_r(0, ",", &tmp);pa.balanceAngle=atof(str);
    str = strtok_r(0, ",", &tmp);pa.accAngle=atof(str);
    Serial.print("ack:");Serial.println(c[0]);
  }else if(c[0]=='f'){
    Serial.print("angle:");Serial.print(pa.falling);Serial.print(',');Serial.print(pa.balanceAngle);Serial.print(',');Serial.println(pa.accAngle);
  }else if(c[0]=='G'){ // scaler update
    char * tmp;
    char * str;
    strtok_r(c, ":", &tmp);
    str = strtok_r(0, ",", &tmp);pa.posScaler=atof(str);
    str = strtok_r(0, ",", &tmp);pa.speedScaler=atof(str);
  }else if(c[0]=='g'){
    Serial.print("scaler:");Serial.print(pa.posScaler);Serial.print(',');Serial.println(pa.speedScaler);
  }else if(c[0]=='H'){
    int dl,dr;
    sscanf(c,"H:%d,%d\n",&dl,&dr);
    pa.deadL = dl;pa.deadR = dr;
  }else if(c[0]=='h'){ // deadband
    Serial.print("deadband:");Serial.print(pa.deadL);Serial.print(',');Serial.println(pa.deadR);
  }
}

/******* some initial process *******/
void initMpu()
{
  //MPU6050_init();
  accelgyro.reset();
  delay(300);
  accelgyro.initialize();
  delay(300);
  accelgyro.setRate(0x7);
  accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  accelgyro.setDLPFMode(MPU6050_DLPF_BW_5);
  delay(300);
  int8_t ret = accelgyro.getMotion6(&acc[0], &acc[1], &acc[2], &gry[0], &gry[1], &gry[2]);
  acc0 = acc[2];
  gry0 = gry[0];
}

void initParam()
{
  pa.p = 22;
  pa.i = 136.72;
  pa.d = 1.3;
  pa.output_min = -150;
  pa.output_max = 150;
  pa.sample_time = 10; // milliseconds
  pa.accAngle = 1.0;
  pa.posScaler = 0.0;
  pa.speedScaler = 0.0;
  pa.falling = 45;
  pa.balanceAngle = -0.8;
  pa.deadL = 0.0;
  pa.deadR = 0.0;
  pa.accTime = 100;
}


/******* calculate angle *******/
#define GYROSCOPE_SENSITIVITY 16.384 // 2000 sensitive / 32768
void calcAngle()
{
  int8_t ret = accelgyro.getMotion6(&acc[0], &acc[1], &acc[2], &gry[0], &gry[1], &gry[2]);
  if(ret==-1){
    // occasionally i2c peripheral may go fault by interference form encoder driver, reset it and things should go fine
    Serial.println("*******************************************");
    Fastwire::setup(150, true);
  }else{
    //angleXY = atan2((double)acc[0], -(double)acc[1]) * 180 / M_PI;
    //gyroZ = -((float)gry[2] / GYROSCOPE_SENSITIVITY);
    if(abs(acc[2]-acc0)<10000){
      angleXY = atan2((double)acc[1], (double)acc[2]) * 180 / M_PI;
      gyroZ = ((float)gry[0] / GYROSCOPE_SENSITIVITY);
      Kalman_Filter(angleXY,gyroZ);
      acc0 = acc[2];
      //gry0 = gry[0];
    }
  }
}

/**** calculate speed and position ****/
// NOTE: wheel R in inverse direction
int posUpdateDelay=0;
void calcPosAndSpeed()
{
  if(posUpdateDelay++>=9){
    float spdL,spdR;
    //motor.getPosition(&posR,&posL);
    if(false==(posR==posR) || false==(posL==posL)){
      resetEncoder();
      Serial.println("NaN");
      return;
    }
    spdR = -(posR-posR0);
    spdL = posL-posL0;
    speed = speed*0.7+(spdR+spdL)*0.3;
    position = position + speed;
    posR0 = posR;
    posL0 = posL;
    posUpdateDelay = 0;
  }
}

void resetEncoder()
{
  //motor.reset();
  speed = 0;
  position = 0;
  posR0 = 0;
  posL0 = 0;
}

/******* calculate pwm *******/
void calcPwm()
{
  // todo: update setangle from encoder
  setAngle = pa.balanceAngle;
//  setAngle += speed*pa.speedScaler;
//  setAngle += position*pa.posScaler;
  setAngle = constrain(setAngle,-10.0f,10.0f);
  setAngle = constrain(setAngle,setAngleLast-0.1,setAngleLast+0.1);
  setAngleLast = setAngle;
  //pwm=calcPid(Angle,setAngle);
  pwm=calcPidIncr(Angle,setAngle);
  
  // fix for deadband
  // motor left inverse direction
  if(pwm>=0){
    pwmR = pwm+pa.deadR;
    pwmL = -pwm-pa.deadL;
  }else{
    pwmR = pwm-pa.deadR;
    pwmL = -pwm+pa.deadL;  
  }
  pwmR+=userTurn;
  pwmL+=userTurn;
  pwmR=constrain(pwmR,pa.output_min,pa.output_max);
  pwmL=constrain(pwmL,pa.output_min,pa.output_max);
  if(en){
    //motor.RunPwm(pwmR,pwmL); 
    m1.run(pwmR);m2.run(pwmL);
  }else{
    //motor.RunPwm(0.0f,0.0f);
    m1.run(0);m2.run(0);
  }
}

/******* read debug command *******/
char cmd[64];
char cmdindex;
void readSerial()
{
  if(Serial.available()){
    char c = Serial.read();
    if(c=='\n'){
      parseCommand(cmd);
      memset(cmd,0,64);
      cmdindex=0;
    }else{
      cmd[cmdindex++]=c;
    }
  }
}

/******* parse infra cmd *******/
void parseInfra()
{
  if(userAccCnt){
    userAccCnt--;
    if(userAccCnt==0){
      userAcc = 0;
    }
  }
  if(userTurnCnt){
    userTurnCnt--;
    if(userTurnCnt==0){
      userTurn = 0;
    }
  }
  
  if(ira.available())
  {
    switch(ira.read())
    {
        case IR_BUTTON_POWER: 
          //Serial.println("Press Power.");
          break;
        case IR_BUTTON_PLUS: 
          //Serial.println("Press Plus.");
          userAcc = -pa.accAngle;
          userAccCnt = pa.accTime;
          break;
        case IR_BUTTON_PREVIOUS: 
          //Serial.println("Press Previous.");
          userTurn = -15;
          userTurnCnt = 100;
          break;
        case IR_BUTTON_PLAY: 
          //Serial.println("Press Play.");
          userAcc = 0;
          userAccCnt = 0;
          userTurn = 0;
          userTurnCnt = 0;
          break;
        case IR_BUTTON_NEXT: 
          //Serial.println("Press Next.");
          userTurn = 15;
          userTurnCnt = 100;
          break;
        case IR_BUTTON_MINUS: 
          //Serial.println("Press Minus.");
          userAcc = pa.accAngle;
          userAccCnt = pa.accTime;
          break;
        case IR_BUTTON_1: 
          //Serial.println("Press Minus.");
          pa.balanceAngle-=0.1;
          break;
        case IR_BUTTON_2: 
          //Serial.println("Press Minus.");
          pa.balanceAngle+=0.1;
          break;
        default:break;
    }
  }
}

/******* print debug msg *******/
void printDebug()
{
  if(debug){
    Serial.print("dbg:");
    Serial.print(Angle);Serial.print(',');
    Serial.print(pwm);Serial.print(',');
    Serial.print(PTerm);Serial.print(',');
    //Serial.print(pid.ITerm);Serial.print(',');Serial.print(pid.DTerm);Serial.print(',');Serial.println(setpoint);
    Serial.print(ITerm);Serial.print(',');Serial.print(DTerm);Serial.print(',');Serial.println(position);
  }
}

void setup() {
  Serial.begin(115200);
  Fastwire::setup(150, true);
  initParam();
  initMpu();
  ira.begin();
  resetEncoder();
  for(int i=0;i<50;i++){
    calcAngle(); // let the filter run for a while
  }
}

void loop() {
  readSerial();
  calcAngle();
  //calcPosAndSpeed();
  calcPwm();
  printDebug();
  dbgPortSwitch();
  parseInfra();
  if(en==1 && abs(Angle)>pa.falling){
      en = 0;
      resetPid();
      Serial.print("en:");Serial.print(debug);Serial.print(',');Serial.println(en);
  }
  if(en==0 && abs(Angle)<5){ // auto recover
      en = 1;
      resetEncoder();resetPid();
      Serial.print("en:");Serial.print(debug);Serial.print(',');Serial.println(en);
  }
  fixdelay();
}
