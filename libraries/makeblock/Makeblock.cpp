#include "Makeblock.h"
#include "I2Cdev.h"

#define MeBaseBoard


#if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU

MePort_Sig mePort[11] = {{NC, NC}, {11, A8}, {13, A11}, {A10, A9}, {1, 0},
    {MISO, SCK}, {A0, A1}, {A2, A3}, {A4, A5}, {6, 7}, {5, 4}
};
#else // else ATmega328
MePort_Sig mePort[11] = {{NC, NC}, {11, 10}, {3, 9}, {12, 13}, {8, 2},
    {NC, NC}, {A2, A3}, {NC, A1}, {NC, A0}, {6, 7}, {5, 4}
};

#endif

union{
    byte b[4];
    float fVal;
    long lVal;
}u;

/*        Port       */
MePort::MePort(){
	s1 = mePort[0].s1;
    s2 = mePort[0].s2;
    _port = 0;
}
MePort::MePort(uint8_t port)
{
    s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
	//The PWM frequency is 976 Hz
#if defined(__AVR_ATmega32U4__) //MeBaseBoard use ATmega32U4 as MCU

TCCR1A =  _BV(WGM10);
TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

TCCR3A = _BV(WGM30);
TCCR3B = _BV(CS31) | _BV(CS30) | _BV(WGM32);

TCCR4B = _BV(CS42) | _BV(CS41) | _BV(CS40);
TCCR4D = 0;

#else if defined(__AVR_ATmega328__) // else ATmega328

TCCR1A = _BV(WGM10);
TCCR1B = _BV(CS11) | _BV(CS10) | _BV(WGM12);

TCCR2A = _BV(WGM21) |_BV(WGM20);
TCCR2B = _BV(CS22);

#endif
}
uint8_t MePort::getPort(){
	return _port;
}
uint8_t MePort::getSlot(){
	return _slot;
}
bool MePort::Dread1()
{
    bool val;
    pinMode(s1, INPUT);
    val = digitalRead(s1);
    return val;
}

bool MePort::Dread2()
{
    bool val;
	pinMode(s2, INPUT);
    val = digitalRead(s2);
    return val;
}

void MePort::Dwrite1(bool value)
{
    pinMode(s1, OUTPUT);
    digitalWrite(s1, value);
}

void MePort::Dwrite2(bool value)
{
    pinMode(s2, OUTPUT);
    digitalWrite(s2, value);
}

int MePort::Aread1()
{
    int val;
    val = analogRead(s1);
    return val;
}

int MePort::Aread2()
{
    int val;
    val = analogRead(s2);
    return val;
}

void MePort::Awrite1(int value)
{   
    analogWrite(s1, value);  
}

void MePort::Awrite2(int value)
{
    analogWrite(s2, value); 
}
void MePort::reset(uint8_t port){
	s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
}
void MePort::reset(uint8_t port,uint8_t slot){
	s1 = mePort[port].s1;
    s2 = mePort[port].s2;
    _port = port;
    _slot = slot;
}
uint8_t MePort::pin1(){
	return s1;
}
uint8_t MePort::pin2(){
	return s2;
}
/*             Wire               */
MeWire::MeWire(uint8_t address): MePort(){
	_slaveAddress = address + 1;
}
MeWire::MeWire(uint8_t port, uint8_t address): MePort(port)
{
    _slaveAddress = address + 1;
}
void MeWire::begin()
{
	if(!isWireBegin){
		isWireBegin = true;
		delay(1000);
		Wire.begin();		
		//TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz
	}
    write(BEGIN_FLAG, 0x01);
}
bool MeWire::isRunning()
{
    return read(BEGIN_STATE);
}
void MeWire::setI2CBaseAddress(uint8_t baseAddress)
{
    byte w[2]={0};
    byte r[4]={0};
    w[0]=0x21;
    w[1]=baseAddress;
    request(w,r,2,4);
}

byte MeWire::read(byte dataAddress){
	byte *b={0};
	read(dataAddress,b,1);
	return b[0];
}

void MeWire::read(byte dataAddress,uint8_t *buf,int len)
{
	byte rxByte;
	Wire.beginTransmission(_slaveAddress); // transmit to device
	Wire.write(dataAddress); // sends one byte
	Wire.endTransmission(); // stop transmitting
	//delayMicroseconds(1);
	Wire.requestFrom(_slaveAddress,len); // request 6 bytes from slave device
	int index =0;
#if 0
	while(Wire.available()) // slave may send less than requested
	{
		rxByte = Wire.read(); // receive a byte as character
		buf[index] = rxByte;
		index++;
	}
#else
	uint16_t timeout = 200;
	uint32_t t1 = millis();
	for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); index++) {
		buf[index] = Wire.read();
	}
	//Wire.endTransmission();
#endif
}

void MeWire::write(byte dataAddress, byte data)
{
    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(dataAddress); // sends one byte
    Wire.endTransmission(); // stop transmitting

    Wire.beginTransmission(_slaveAddress); // transmit to device
    Wire.write(data); // sends one byte
    Wire.endTransmission(); // stop transmitting
}
void MeWire::request(byte* writeData,byte*readData,int wlen,int rlen)
{
   
	uint8_t rxByte;
	uint8_t index =0;

	Wire.beginTransmission(_slaveAddress); // transmit to device

	Wire.write(writeData,wlen);

	Wire.endTransmission(); 
	//delayMicroseconds(2);
	Wire.requestFrom(_slaveAddress,rlen); // request 6 bytes from slave device
	//delayMicroseconds(2);
#if 1
	while(Wire.available()) // slave may send less than requested
	{
		rxByte = Wire.read(); // receive a byte as character
        
		readData[index] = rxByte;
		index++; 
	}
#else
	uint16_t timeout = 200;
	uint32_t t1 = millis();
	for (; Wire.available() && (timeout == 0 || millis() - t1 < timeout); index++) {
		readData[index] = Wire.read();
	}	
	//Wire.endTransmission();
#endif
}
/*      MeParams       */
MeParams::MeParams()
{
    _root = createObject();
    memset(_root->child, 0, sizeof(MeParamObject));
}
void MeParams::parse(char* s){
	clear();
  char *p=NULL;
  char *v=NULL;
  p = (char*)malloc(20*sizeof(char));
  v = (char*)malloc(40*sizeof(char));
  int i;
  int len = strlen(s);
  int pIndex = 0;
  int vIndex = 0;
  bool pEnd = false;
  bool vEnd = true;
  for(i=0;i<len;i++){
    if(s[i]=='&'){
      pEnd = false;
      vEnd = true;
      setParam(p,v);
      memset(p,0,20);
      memset(v,0,40);
      pIndex = i+1;
    }
    if(s[i]=='='){
      pEnd=true;
      vEnd = false;
      vIndex = i+1;
    }
    if(vEnd==false){
      if(i-vIndex>=0){
        v[i-vIndex]=s[i];
      } 
    }
    if(pEnd==false){
      if(i-pIndex>=0){
       p[i-pIndex]=s[i];
      }
    }
  }
  setParam(p,v);
  memset(p,0,20);
  memset(v,0,40);
  free(p);
  free(v);
}
MeParamObject *MeParams::getParam(const char *string)
{
    MeParamObject *c = _root->child;
    while (c && strcasecmp(c->name, string))
        c = c->next;
    return c;
}
void MeParams::setParam(char *name, char *n)
{
	bool isStr = true;
	double v = atof(n);	
	isStr = v==0;
	int i=0;
	int len = strlen(n);
	for(i=0;i<len;i++){
		if(i==0){
			if(n[i]==43||n[i]==45){
				continue;
			}
		}
		if(n[i]==46){
			continue;
		}
		if(!(n[i]>=48&&n[i]<=57)){
			isStr = true;	
			break;
		}
	}
    deleteParam(name);
    if(isStr) {
        addItemToObject(name, createCharItem(n));
    } else {
        addItemToObject(name, createItem(v));
    }
}
double MeParams::getParamValue(const char *string)
{
    return getParam(string)->value;
}
char *MeParams::getParamCode(const char *string)
{
    return getParam(string)->code;
}
void MeParams::clear()
{
    unsigned char i = 0;
    MeParamObject *c = _root->child;
    MeParamObject *prev;
    while (c){
		prev = c;
    	c = c->next;
    }
    c = prev;
    while(prev){
    	c = prev->prev;
		deleteParam(prev->name);
    	prev = c;
    }

}
void MeParams::deleteParam(char *string)
{	
    deleteItemFromRoot(detachItemFromObject(string));
}
MeParamObject *MeParams::createObject()
{
    MeParamObject *item = (MeParamObject *) malloc(sizeof(MeParamObject));
    if (item) {
        memset(item, 0, sizeof(MeParamObject));
    }
    return item;
}
MeParamObject *MeParams::createItem(double n)
{
    MeParamObject *item = (MeParamObject *) malloc(sizeof(MeParamObject));
    if (item) {
        memset(item, 0, sizeof(MeParamObject));
        item->value = n;
        item->type = 1;
    }
    return item;
}
MeParamObject *MeParams::createCharItem(char *n)
{
    MeParamObject *item = (MeParamObject *) malloc(sizeof(MeParamObject));
    if (item) {
        memset(item, 0, sizeof(MeParamObject));
        item->code = strdup(n);
        item->type = 2;
    }
    return item;
}

void MeParams::addItemToObject(char *string, MeParamObject *item)
{
    if (!item)
        return;
    if (item->name){
    	free(item->name);
    }
        
    item->name = strdup(string);
    MeParamObject *c = _root->child;
    if (!item)
        return;

    if (!c) {
        _root->child = item;
    } else {
        while (c && c->next)
            c = c->next;
        suffixObject(c, item);
    }
}
void MeParams::deleteItemFromRoot(MeParamObject *c)
{
    MeParamObject *next;
    while (c) {
        next = c->next;
        if (c->name) {
            free(c->name);
        }
        if (c->child) {
            deleteItemFromRoot(c->child);
        }
        if(c->code&&c->type==2){
			free(c->code);
		}
		c->type=0;
        free(c);
        c = next;
    }
}
MeParamObject *MeParams::detachItemFromObject( char *string)
{
    unsigned char i = 0;
    MeParamObject *c = _root->child;
    while (c && strcasecmp(c->name, string))
        i++, c = c->next;
    if (c)
        return detachItemFromArray(i);
    return 0;
}
void MeParams::deleteItemFromArray(unsigned char which)
{
    deleteItemFromRoot(detachItemFromArray(which));
}
MeParamObject *MeParams::detachItemFromArray(unsigned char which)
{
    MeParamObject *c = _root->child;
    while (c && which > 0)
        c = c->next, which--;
    if (!c)
        return 0;

    if (c->prev)
        c->prev->next = c->next;
    if (c->next)
        c->next->prev = c->prev;
    if (c == _root->child)
        _root->child = c->next;
    c->prev = c->next = 0;
    return c;
}

void MeParams::suffixObject(MeParamObject *prev, MeParamObject *item)
{
    prev->next = item;
    item->prev = prev;
}

/*             Serial                  */
MeSerial::MeSerial():MePort(),SoftwareSerial(NC,NC){
    _hard = true;
    _scratch = true;
    _polling = false;
}
MeSerial::MeSerial(uint8_t port):MePort(port),SoftwareSerial(mePort[port].s2,mePort[port].s1)
{
	_scratch = false;
    _hard = false;
    _polling = false;
    #if defined(__AVR_ATmega32U4__)
        _polling = getPort()>PORT_5;
        _hard = getPort()==PORT_4;
    #else
    	_hard = getPort()==PORT_5;    
    #endif
}
void MeSerial::setHardware(bool mode){
    _hard = mode;
}
void MeSerial::begin(long baudrate)
{
    _bitPeriod = 1000000/baudrate;
    if(_hard) {
		#if defined(__AVR_ATmega32U4__)
            _scratch?Serial.begin(baudrate):Serial1.begin(baudrate);
        #else
            Serial.begin(baudrate);
		#endif
    } else {
        SoftwareSerial::begin(baudrate);
    }
}
size_t MeSerial::write(uint8_t byte)
{
    if(_isServoBusy == true)return -1;
    if(_hard){
    	#if defined(__AVR_ATmega32U4__)
            return (_scratch?Serial.write(byte):Serial1.write(byte));
        #else
            return Serial.write(byte);
		#endif
    }else return SoftwareSerial::write(byte);
}
int MeSerial::read()
{
    if(_isServoBusy == true)return -1;
    
    if(_polling){
        int temp = _byte;
        _byte = -1;
        return temp>-1?temp:poll();
    }
    if(_hard){
		#if defined(__AVR_ATmega32U4__)
        	return (_scratch?Serial.read():Serial1.read());
        #else
        	return Serial.read();
		#endif
    }else return SoftwareSerial::read();
}
int MeSerial::available()
{
    if(_polling){
        _byte = poll();
        return _byte>-1?1:0;
    }
    if(_hard){
    	#if defined(__AVR_ATmega32U4__)
        	return (_scratch?Serial.available():Serial1.available());
        #else
        	return Serial.available();
		#endif
    }else return SoftwareSerial::available();
}
bool MeSerial::listen()
{
    if(_hard)
        return true;
    else return SoftwareSerial::listen();
}
bool MeSerial::isListening()
{
    if(_hard)
        return true;
    else return SoftwareSerial::isListening();
}
int MeSerial::poll()
{
    int val = 0;
    int bitDelay = _bitPeriod - clockCyclesToMicroseconds(50);
    if (digitalRead(s2) == LOW) {
        for (int offset = 0; offset < 8; offset++) {
            delayMicroseconds(bitDelay);
            val |= digitalRead(s2) << offset;
        }
        delayMicroseconds(bitDelay);
        return val&0xff;
    }
    return -1;
}
bool MeSerial::paramAvailable()
{
    bool isParse = (millis()-_lastTime)>100&&_index>0;
    if(this->available()) {
        char c = this->read();
        if(c == '\n'||c == '\r'||isParse) {
        	if(_index<3){
	        	return false;
	        }
            _cmds[_index] = '\0';
            char str[_index];
            strcpy(str, _cmds);
			int i=0;
            for(i=0;i<_index;i++){
            	if(str[i]=='='&&i>0){
		            _params.parse(str);
					_index = 0;
					return true;
	            }
            }
			_index = 0;
			return false;
        } else {
            _cmds[_index] = c;
            _index++;
        }
        
    	_lastTime = millis();
    }
    return false;
}

double MeSerial::getParamValue(char *str)
{
    return _params.getParamValue(str);
}
char *MeSerial::getParamCode(char *str)
{
    return _params.getParamCode(str);
}
MeParams MeSerial::getParams()
{
    return _params;
}
void MeSerial::findParamName(char *str, int len)
{
    byte i = 0;
    for(i = 0; i < len; i++) {
        if(str[i] == '=') {
            char name[i+1];
            memcpy(name, str, i);
            name[i] = '\0';
            char s[len];
            int j;
            for(j = i + 1; j < len; j++) {
                s[j-i-1] = str[j];
            }
            findParamValue(s, len - i - 1, name);
            break;
        }
    }
}
void MeSerial::findParamValue(char *str, int len, char *name)
{
    byte i = 0;
    for(i = 0; i < len; i++) {
        if(str[i] == '&' || str[i] == '\0' || i == len - 1) {
            char v[i+1];
            memcpy(v, str, i);
            v[i] = '\0';

            _params.setParam(name, v);
            if(i < len - 1) {
                char s[len];
                int j;
                for(j = i + 1; j < len; j++) {
                    s[j-i-1] = str[j];
                }
                findParamName(s, len - i - 1);
                break;
            }
        }
    }
}
/*             LineFinder              */
MeLineFinder::MeLineFinder(): MePort(0){
	
}
MeLineFinder::MeLineFinder(uint8_t port): MePort(port)
{

}
int MeLineFinder::readSensors()
{
    int state = S1_IN_S2_IN;
    int s1State = MePort::Dread1();
    int s2State = MePort::Dread2();
    state = ((1 & s1State) << 1) | s2State;
    return state;
}
int MeLineFinder::readSensor1()
{
    return MePort::Dread1();
}
int MeLineFinder::readSensor2()
{
    return MePort::Dread2();
}
/*             LimitSwitch              */
MeLimitSwitch::MeLimitSwitch(): MePort(0)
{
}
MeLimitSwitch::MeLimitSwitch(uint8_t port): MePort(port)
{
    _device = DEV1;
    pinMode(s2,INPUT_PULLUP);
}
MeLimitSwitch::MeLimitSwitch(uint8_t port,uint8_t slot): MePort(port)
{
    reset(port,slot);
    if(getSlot()==DEV2){
        pinMode(s1,INPUT_PULLUP);
    }else{
        pinMode(s2,INPUT_PULLUP);
    }
}
bool MeLimitSwitch::touched()                                                                                                                                                          
{
    if(getSlot()==DEV2){
        pinMode(s1,INPUT_PULLUP);
    }else{
        pinMode(s2,INPUT_PULLUP);
    }
    return getSlot()==DEV2?digitalRead(s1):digitalRead(s2);
}

/*             MotorDriver              */
MeDCMotor::MeDCMotor(): MePort(0)
{

}
MeDCMotor::MeDCMotor(uint8_t port): MePort(port)
{

}
void MeDCMotor::run(int speed)
{
    speed = speed > 255 ? 255 : speed;
    speed = speed < -255 ? -255 : speed;

    if(speed >= 0) {
        MePort::Dwrite2(HIGH);
        MePort::Awrite1(speed);
    } else {
        MePort::Dwrite2(LOW);
        MePort::Awrite1(-speed);
    }
}
void MeDCMotor::stop()
{
    MeDCMotor::run(0);
}
/*           UltrasonicSenser                 */
MeUltrasonicSensor::MeUltrasonicSensor(): MePort(0)
{
}

MeUltrasonicSensor::MeUltrasonicSensor(uint8_t port): MePort(port)
{
}

long MeUltrasonicSensor::distanceCm()
{
    long distance = measure();
    return ((distance / 29) >> 1);
}

long MeUltrasonicSensor::distanceInch()
{
    long distance = measure();
    return ((distance / 74) >> 1);
}

long MeUltrasonicSensor::measure()
{
    long duration;
    MePort::Dwrite2(LOW);
    delayMicroseconds(2);
    MePort::Dwrite2(HIGH);	
    delayMicroseconds(10);
    MePort::Dwrite2(LOW);
    pinMode(s2, INPUT);
    duration = pulseIn(s2, HIGH, 500000); 
    return duration;
}

/*          shutter       */
MeShutter::MeShutter(): MePort(0){
	
}
MeShutter::MeShutter(uint8_t port): MePort(port)
{
    MePort::Dwrite1(LOW);
    MePort::Dwrite2(LOW);
}
void MeShutter::shotOn()
{
    MePort::Dwrite1(HIGH);
}
void MeShutter::shotOff()
{

    MePort::Dwrite1(LOW);
}
void MeShutter::focusOn()
{
    MePort::Dwrite2(HIGH);
}
void MeShutter::focusOff()
{
    MePort::Dwrite2(LOW);
}

/*           Bluetooth                 */
MeBluetooth::MeBluetooth(): MeSerial(0)
{
}
MeBluetooth::MeBluetooth(uint8_t port): MeSerial(port)
{
}

/*           Infrared Receiver                 */
MeInfraredReceiver::MeInfraredReceiver(): MeSerial(0)
{
	
}
MeInfraredReceiver::MeInfraredReceiver(uint8_t port): MeSerial(port)
{
}
void MeInfraredReceiver::begin()
{
    MeSerial::setHardware(false);
    MeSerial::begin(9600);
}
bool MeInfraredReceiver::buttonState()        // Not available in Switching mode
{
    return !(MePort::Dread1());
}

MeRGBLed::MeRGBLed():MePort(0) {
	setNumber(4);
}
MeRGBLed::MeRGBLed(uint8_t port):MePort(port) {
	pinMask = digitalPinToBitMask(s2);
	ws2812_port = portOutputRegister(digitalPinToPort(s2));
	ws2812_port_reg = portModeRegister(digitalPinToPort(s2));
	setNumber(4);
}
MeRGBLed::MeRGBLed(uint8_t port,uint8_t slot):MePort(port){
	if(slot==DEV1){
		pinMask = digitalPinToBitMask(s2);
		ws2812_port = portOutputRegister(digitalPinToPort(s2));
		ws2812_port_reg = portModeRegister(digitalPinToPort(s2));
	}else{
		pinMask = digitalPinToBitMask(s1);
		ws2812_port = portOutputRegister(digitalPinToPort(s1));
		ws2812_port_reg = portModeRegister(digitalPinToPort(s1));
	}
	setNumber(4);
}
void MeRGBLed::reset(uint8_t port){
	s2 = mePort[port].s2;
	s1 = mePort[port].s1;
	pinMask = digitalPinToBitMask(s2);
	ws2812_port = portOutputRegister(digitalPinToPort(s2));
	ws2812_port_reg = portModeRegister(digitalPinToPort(s2));
}
void MeRGBLed::reset(uint8_t port,uint8_t slot){
	s2 = mePort[port].s2;
	s1 = mePort[port].s1;
	if(slot==DEV1){
		pinMask = digitalPinToBitMask(s2);
		ws2812_port = portOutputRegister(digitalPinToPort(s2));
		ws2812_port_reg = portModeRegister(digitalPinToPort(s2));
	}else{
		pinMask = digitalPinToBitMask(s1);
		ws2812_port = portOutputRegister(digitalPinToPort(s1));
		ws2812_port_reg = portModeRegister(digitalPinToPort(s1));
	}
}
void MeRGBLed::setNumber(uint8_t num_leds){
	count_led = num_leds;
	pixels = (uint8_t*)malloc(count_led*3);
}
cRGB MeRGBLed::getColorAt(uint8_t index) {
	
	cRGB px_value;
	
	if(index < count_led) {
		
		uint8_t tmp;
		tmp = index * 3;
		
		px_value.g = pixels[tmp];
		px_value.r = pixels[tmp+1];
		px_value.b = pixels[tmp+2];
	}
	
	return px_value;
}

uint8_t MeRGBLed::getNumber(){
	return count_led;
}
bool MeRGBLed::setColorAt(uint8_t index, uint8_t red,uint8_t green,uint8_t blue) {
	if(index < count_led) {
		uint8_t tmp = index * 3;
		pixels[tmp] = green;
		pixels[tmp+1] = red;
		pixels[tmp+2] = blue;
		
		return true;
	} 
	return false;
}
bool MeRGBLed::setColorAt(uint8_t index, long value) {
	if(index < count_led) {
		uint8_t tmp = index * 3;
		uint8_t red = (value&0xff0000)>>16;
		uint8_t green = (value&0xff00)>>8;
		uint8_t blue = value&0xff;
		pixels[tmp] = green;
		pixels[tmp+1] = red;
		pixels[tmp+2] = blue;
		return true;
	} 
	return false;
}
void MeRGBLed::clear(){
	for(int i=0;i<getNumber();i++){
		setColorAt(i, 0);
	}
	show();
}
/*
  This routine writes an array of bytes with RGB values to the Dataout pin
  using the fast 800kHz clockless WS2811/2812 protocol.
*/

// Timing in ns
#define w_zeropulse   350
#define w_onepulse    900
#define w_totalperiod 1250

// Fixed cycles used by the inner loop
#define w_fixedlow    3
#define w_fixedhigh   6
#define w_fixedtotal  10   

// Insert NOPs to match the timing, if possible
#define w_zerocycles    (((F_CPU/1000)*w_zeropulse          )/1000000)
#define w_onecycles     (((F_CPU/1000)*w_onepulse    +500000)/1000000)
#define w_totalcycles   (((F_CPU/1000)*w_totalperiod +500000)/1000000)

// w1 - nops between rising edge and falling edge - low
#define w1 (w_zerocycles-w_fixedlow)
// w2   nops between fe low and fe high
#define w2 (w_onecycles-w_fixedhigh-w1)
// w3   nops to complete loop
#define w3 (w_totalcycles-w_fixedtotal-w1-w2)

#if w1>0
  #define w1_nops w1
#else
  #define w1_nops  0
#endif

// The only critical timing parameter is the minimum pulse length of the "0"
// Warn or throw error if this timing can not be met with current F_CPU settings.
#define w_lowtime ((w1_nops+w_fixedlow)*1000000)/(F_CPU/1000)
#if w_lowtime>550
   #error "Light_ws2812: Sorry, the clock speed is too low. Did you set F_CPU correctly?"
#elif w_lowtime>450
   #warning "Light_ws2812: The timing is critical and may only work on WS2812B, not on WS2812(S)."
   #warning "Please consider a higher clockspeed, if possible"
#endif   

#if w2>0
#define w2_nops w2
#else
#define w2_nops  0
#endif

#if w3>0
#define w3_nops w3
#else
#define w3_nops  0
#endif

#define w_nop1  "nop      \n\t"
#define w_nop2  "rjmp .+0 \n\t"
#define w_nop4  w_nop2 w_nop2
#define w_nop8  w_nop4 w_nop4
#define w_nop16 w_nop8 w_nop8

void  MeRGBLed::rgbled_sendarray_mask(uint8_t *data,uint16_t datlen,uint8_t maskhi,uint8_t *port, uint8_t *portreg)
{
  uint8_t curbyte,ctr,masklo;
  uint8_t sreg_prev;
  
  masklo = ~maskhi & *portreg;
  maskhi |= *portreg;
  sreg_prev=SREG;
  cli();  

  while (datlen--) {
    curbyte=*data++;
    
    asm volatile(
    "       ldi   %0,8  \n\t"
    "loop%=:            \n\t"
    "       st    X,%3 \n\t"    //  '1' [02] '0' [02] - re
#if (w1_nops&1)
w_nop1
#endif
#if (w1_nops&2)
w_nop2
#endif
#if (w1_nops&4)
w_nop4
#endif
#if (w1_nops&8)
w_nop8
#endif
#if (w1_nops&16)
w_nop16
#endif
    "       sbrs  %1,7  \n\t"    //  '1' [04] '0' [03]
    "       st    X,%4 \n\t"     //  '1' [--] '0' [05] - fe-low
    "       lsl   %1    \n\t"    //  '1' [05] '0' [06]
#if (w2_nops&1)
  w_nop1
#endif
#if (w2_nops&2)
  w_nop2
#endif
#if (w2_nops&4)
  w_nop4
#endif
#if (w2_nops&8)
  w_nop8
#endif
#if (w2_nops&16)
  w_nop16 
#endif
    "       brcc skipone%= \n\t"    //  '1' [+1] '0' [+2] - 
    "       st   X,%4      \n\t"    //  '1' [+3] '0' [--] - fe-high
    "skipone%=:               "     //  '1' [+3] '0' [+2] - 

#if (w3_nops&1)
w_nop1
#endif
#if (w3_nops&2)
w_nop2
#endif
#if (w3_nops&4)
w_nop4
#endif
#if (w3_nops&8)
w_nop8
#endif
#if (w3_nops&16)
w_nop16
#endif

    "       dec   %0    \n\t"    //  '1' [+4] '0' [+3]
    "       brne  loop%=\n\t"    //  '1' [+5] '0' [+4]
    :	"=&d" (ctr)
//    :	"r" (curbyte), "I" (_SFR_IO_ADDR(ws2812_PORTREG)), "r" (maskhi), "r" (masklo)
    :	"r" (curbyte), "x" (port), "r" (maskhi), "r" (masklo)
    );
  }
  
  SREG=sreg_prev;
}
void MeRGBLed::show() {
	*ws2812_port_reg |= pinMask; // Enable DDR
	rgbled_sendarray_mask(pixels,3*count_led,pinMask,(uint8_t*) ws2812_port,(uint8_t*) ws2812_port_reg );	
}

MeRGBLed::~MeRGBLed() {
	
	
}
#if 0
/*          EncoderMotor        */

MeEncoderMotor::MeEncoderMotor(uint8_t selector,uint8_t slot):MeWire(selector){
    _slot = slot;
}
void MeEncoderMotor::begin(){
	MeWire::begin();
	resetEncoder();
}
boolean MeEncoderMotor::setCounter(uint8_t counter){
    byte w[5]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x23;
    w[2]=0;
    w[3]=_slot;
    w[4]=counter;
    request(w,r,5,4);
    return r[3]==1;
}
boolean MeEncoderMotor::setRatio(float ratio){
    byte w[7]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x22;
    w[2]=_slot;
    u.fVal = ratio;
    w[3]=u.b[0];
    w[4]=u.b[1];
    w[5]=u.b[2];
    w[6]=u.b[3];
    request(w,r,7,4);
    return r[3]==1;
}
boolean MeEncoderMotor::setPID(float mp,float mi,float md,uint8_t mode){
    
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x24;
    w[2]=_slot;
    w[4]=mode;
    
    int i;
    
    w[3]=0;
    u.fVal = mp;
    for(i=0;i<4;i++){
        w[5+i]=u.b[i];
    }
    request(w,r,9,4);
    
    w[3]=1;
    u.fVal = mi;
    for(i=0;i<4;i++){
        w[5+i]=u.b[i];
    }
    request(w,r,9,4);
    
    w[3]=2;
    u.fVal = md;
    for(i=0;i<4;i++){
        w[5+i]=u.b[i];
    }
    request(w,r,9,4);
    return r[3]==1;
}
boolean MeEncoderMotor::moveTo(long degrees,float speed){
	byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x32;
    w[2]=_slot;
    int i;
    u.lVal = degrees;
    for(i=0;i<4;i++){
        w[3+i]=u.b[i];
    }
    u.fVal = speed;
    for(i=0;i<4;i++){
        w[7+i]=u.b[i];
    }
    request(w,r,11,4);
    return r[3]==1;
}
boolean MeEncoderMotor::move(long degrees,float speed){
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x31;
    w[2]=_slot;
    int i;
    u.lVal = degrees;
    for(i=0;i<4;i++){
        w[3+i]=u.b[i];
    }
    u.fVal = speed;
    for(i=0;i<4;i++){
        w[7+i]=u.b[i];
    }
    request(w,r,11,4);
    return r[3]==1;
}
boolean MeEncoderMotor::runTurns(float turns){
    return move(turns*360,10);
}
boolean MeEncoderMotor::runSpeed(float speed){
	return runSpeedAndTime(speed,0);
}
boolean MeEncoderMotor::runSpeedAndTime(float speed,long time){
    byte w[9]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x33;
    w[2]=_slot;
    int i;
    u.fVal = speed;
    for(i=0;i<4;i++){
        w[3+i]=u.b[i];
    }
    u.lVal = time;
    for(i=0;i<4;i++){
        w[7+i]=u.b[i];
    }
    request(w,r,11,4);
    return r[3]==1;
}

boolean MeEncoderMotor::resetEncoder(){
    byte w[3]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x54;
    w[2]=_slot;
    request(w,r,11,3);
    return r[3]==1;
}
boolean MeEncoderMotor::enableDebug(){
    
    byte w[2]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x55;
    request(w,r,11,2);
    return r[3]==1;
}

boolean MeEncoderMotor::setCommandFlag(boolean flag){
    byte w[4]={0};
    byte r[4]={0};
    w[0]=0x91;
    w[1]=0x41;
    w[2]=_slot;
    w[3]=flag;
    request(w,r,4,4);
    return r[3]==1;
}
float MeEncoderMotor::getCurrentSpeed(){
    byte w[3]={0};
    byte r[7]={0};
    w[0]=0x91;
    w[1]=0x51;
    w[2]=_slot;
    request(w,r,3,7);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[3+i];
    }
    return u.fVal;
}
float MeEncoderMotor::getCurrentPosition(){
    byte w[3]={0};
    byte r[7]={0};
    w[0]=0x91;
    w[1]=0x52;
    w[2]=_slot;
    request(w,r,3,7);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[3+i];
    }
    return u.fVal;
}
float MeEncoderMotor::getPIDParam(uint8_t type,uint8_t mode){
    
    byte w[5]={0};
    byte r[9]={0};
    w[0]=0x91;
    w[1]=0x53;
    w[2]=_slot;
    w[3]=type;
    w[4]=mode;
    request(w,r,5,9);
    int i;
    for (i=0; i<4; i++) {
        u.b[i]=r[5+i];
    }
    return u.fVal;
    
}
#else

/*          EncoderMotor        */
#define USEI2CDEV 1
MeEncoderMotor::MeEncoderMotor(uint8_t _addr)
{
    addr = _addr;
}

void MeEncoderMotor::RunPwm(float pwm0,float pwm1)
{
	byte txBuf[9];
	txBuf[0] = REG_RUN_PWM_BOTH;
	*(float*)&txBuf[1] = pwm0;
    *(float*)&txBuf[5] = pwm1;
#if USEI2CDEV
	I2Cdev::writeBytes(addr, txBuf[0],8,&txBuf[1]);
#else
	Wire.beginTransmission(addr); // transmit to device
	Wire.write(txBuf,9);
	Wire.endTransmission(); // stop transmitting
#endif
}

void MeEncoderMotor::RunPwm(byte slot, float pwm)
{
	byte txBuf[5];
	txBuf[0] = slot==0?REG_RUN_PWM_SLOT0:REG_RUN_PWM_SLOT1;
	*(float*)&txBuf[1] = pwm;

#if USEI2CDEV
	I2Cdev::writeBytes(addr, txBuf[0],4,&txBuf[1]);
#else
	Wire.beginTransmission(addr); // transmit to device
	Wire.write(txBuf,5);
	Wire.endTransmission(); // stop transmitting
#endif
}

void MeEncoderMotor::getPosition(float * pos0,float * pos1)
{
	byte rxBuf[8];
	byte rxCnt=0;
#if USEI2CDEV
	I2Cdev::readBytes(addr, REG_GET_POS_BOTH,8,rxBuf);
#else
	Wire.beginTransmission(addr); // transmit to device
	Wire.write(REG_GET_POS_BOTH);
	Wire.endTransmission(); // stop transmitting
	delayMicroseconds(10);
	Wire.requestFrom((int)addr, 8);
	while(Wire.available()){
	  rxBuf[rxCnt++] = Wire.read();
	}
#endif
	*pos0 = *(float*)&rxBuf[0];
	*pos1 = *(float*)&rxBuf[4];
}

void MeEncoderMotor::getPosition(byte slot,float * pos)
{
	byte rxBuf[4];	
	byte rxCnt=0;
	int reg = slot==0?REG_GET_POS_SLOT0:REG_GET_POS_SLOT1;
#if USEI2CDEV
	I2Cdev::readBytes(addr, reg,4,rxBuf);
#else
	Wire.beginTransmission(addr); // transmit to device
	Wire.write(REG_GET_POS_BOTH);
	Wire.endTransmission(); // stop transmitting
	delayMicroseconds(10);
	Wire.requestFrom((int)addr, 4);
	while(Wire.available()){
	  rxBuf[rxCnt++] = Wire.read();
	}
#endif
	*pos = *(float*)&rxBuf[0];
}

void MeEncoderMotor::reset()
{
	byte txBuf[2];
	txBuf[0] = REG_RESET;
#if USEI2CDEV
	I2Cdev::writeBytes(addr, txBuf[0],1,&txBuf[1]);
#else

#endif

}




#endif

/*          Stepper     */

MeStepperMotor::MeStepperMotor(uint8_t port, uint8_t selector): MeWire(port, selector)
{
	dir=s2;
	pulse=s1;
	pinMode(dir,OUTPUT);
    pinMode(pulse,OUTPUT); 
	digitalWrite(dir,LOW);
	digitalWrite(pulse,LOW);
}



void MeStepperMotor::begin()
{
    MeWire::begin(); // join i2c bus (address optional for master)
    delay(100);
    reset();
	delay(100);
    setCurrentPosition(0);
    delay(100);  
    enable();
    delay(100);   
}

uint8_t MeStepperMotor::STP_I2C_communicate(byte mode,long data,byte rlen)
{
  stepper.motor.cmd1=STEPPER_CMD;
  stepper.motor.cmd2=mode;
  stepper.motor.dev=1;
  stepper.motor.data=data;
  byte* RPM;
  RPM=(byte*)&stepper.c[0];
  MeWire::request(RPM,RPM,7,rlen);
  
  //if((stepper.c[0]==STEPPER_CMD)&&(stepper.c[1]==mode)&&(stepper.c[2]==1))
   if((stepper.c[0]==STEPPER_CMD)&&(stepper.c[1]==mode))
      return 1;
   
   else
      return 0;
}


void MeStepperMotor::setMicroStep(byte microStep)
{
	
	STP_I2C_communicate(STP_SET_MS, (long)microStep,3);
}


void MeStepperMotor::reset()
{
	STP_I2C_communicate(STP_RESET, 0,3);
   
}
void MeStepperMotor::setMaxSpeed(long stepperMaxSpeed)
{
    STP_I2C_communicate(STP_MAX_SPEED,stepperMaxSpeed,3);

}
void MeStepperMotor::setAcceleration(long stepperAcceleration)
{
   STP_I2C_communicate(STP_SET_ACC, stepperAcceleration,3);
    
}
void MeStepperMotor::setSpeed(long stepperSpeed)
{
    STP_I2C_communicate(STP_SET_SPEED, stepperSpeed,3);
   
}
void MeStepperMotor::setCurrentPosition(long stepperCurrentPos)
{
    
	STP_I2C_communicate(STP_SET_POS,stepperCurrentPos,3);
   
}
void MeStepperMotor::setI2Cadd(uint8_t slaveI2Cadd)
{
    
	STP_I2C_communicate(SET_I2C_ADD,(long)slaveI2Cadd,3);
   
}

void MeStepperMotor::moveTo(long stepperMoveTo)
{
   STP_I2C_communicate(STP_MOVE_TO, stepperMoveTo,3);
    
}

void MeStepperMotor::move(long stepperMove)
{
    STP_I2C_communicate(STP_MOVE, stepperMove,3);
   
}

long MeStepperMotor::distanceToGo()
{
     STP_I2C_communicate(GET_CURRENT_DIS,0,7);

    return (stepper.motor.data);
}

long MeStepperMotor::targetPosition()
{
     STP_I2C_communicate(GET_GOAL_POS,0,7);
    return (stepper.motor.data);
}
long MeStepperMotor::currentPosition()
{
    STP_I2C_communicate(GET_CURRENT_POS,0,7);
    
    return (stepper.motor.data);
}

void MeStepperMotor::enable()
{
    STP_I2C_communicate(STP_ENABLE,0,3);
}

void MeStepperMotor::disable()
{
    STP_I2C_communicate(STP_DISABLE,0,3);
}

void MeStepperMotor::run()
{
   STP_I2C_communicate(STP_RUN,0,3);
}

void MeStepperMotor::runSpeed()
{
   STP_I2C_communicate(STP_RUN_SPEED,0,3);
}

void MeStepperMotor::stop()
{
    STP_I2C_communicate(STP_STOP,0,3);
}
void MeStepperMotor::onestep()
{
    STP_I2C_communicate(STP_RUN_ASTP,0,3);
}

/*servo*/

#define usToTicks(_us)    (( clockCyclesPerMicrosecond()* _us) / 8)     // converts microseconds to tick (assumes prescale of 8)  // 12 Aug 2009
#define ticksToUs(_ticks) (( (unsigned)_ticks * 8)/ clockCyclesPerMicrosecond() ) // converts from ticks back to microseconds


#define TRIM_DURATION       2                               // compensation ticks to trim adjust for digitalWrite delays // 12 August 2009

//#define NBR_TIMERS        (MAX_SERVOS / SERVOS_PER_TIMER)

static servo_t servos[MAX_SERVOS];                          // static array of servo structures
static volatile int8_t Channel[_Nbr_16timers ];             // counter for the servo being pulsed for each timer (or -1 if refresh interval)

uint8_t ServoCount = 0;                                     // the total number of attached servos

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER)) // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])            // macro to access servo class by timer and channel

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)  // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)  // maximum value in uS for this servo 

/************ static functions common to all instances ***********************/
static bool isTimerActive(timer16_Sequence_t timer)
{
    // returns true if any servo is active on this timer
    for(uint8_t channel = 0; channel < SERVOS_PER_TIMER; channel++) {
        if(SERVO(timer, channel).Pin.isActive == true)
            return true;
    }
    return false;
}
static void finISR(timer16_Sequence_t timer)
{
    //disable use of the given timer
#if defined WIRING   // Wiring
    if(timer == _timer1) {
#if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
        TIMSK1 &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt
#else
        TIMSK &=  ~_BV(OCIE1A) ;  // disable timer 1 output compare interrupt
#endif
        timerDetach(TIMER1OUTCOMPAREA_INT);
    } else if(timer == _timer3) {
#if defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
        TIMSK3 &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
#else
        ETIMSK &= ~_BV(OCIE3A);    // disable the timer3 output compare A interrupt
#endif
        timerDetach(TIMER3OUTCOMPAREA_INT);
    }
#else
    //For arduino - in future: call here to a currently undefined function to reset the timer
#endif
}
static inline void handle_interrupts(timer16_Sequence_t timer, volatile uint16_t *TCNTn, volatile uint16_t *OCRnA)
{
    if( Channel[timer] < 0 ) {
        *TCNTn = 0; // channel set to -1 indicated that refresh interval completed so reset the timer
        _isServoBusy = false;
    } else {
        if( SERVO_INDEX(timer, Channel[timer]) < ServoCount && SERVO(timer, Channel[timer]).Pin.isActive == true ) {
            digitalWrite( SERVO(timer, Channel[timer]).Pin.nbr, LOW); // pulse this channel low if activated
            _isServoBusy = false;
        }
    }

    Channel[timer]++;    // increment to the next channel
    if( SERVO_INDEX(timer, Channel[timer]) < ServoCount && Channel[timer] < SERVOS_PER_TIMER) {
        *OCRnA = *TCNTn + SERVO(timer, Channel[timer]).ticks;
        if(SERVO(timer, Channel[timer]).Pin.isActive == true) {   // check if activated
            digitalWrite( SERVO(timer, Channel[timer]).Pin.nbr, HIGH); // its an active channel so pulse it high
            _isServoBusy = true;

        }

    } else {

        // finished all channels so wait for the refresh period to expire before starting over
        if( ((unsigned)*TCNTn) + 4 < usToTicks(REFRESH_INTERVAL) )  // allow a few ticks to ensure the next OCR1A not missed
            *OCRnA = (unsigned int)usToTicks(REFRESH_INTERVAL);
        else
            *OCRnA = *TCNTn + 4;  // at least REFRESH_INTERVAL has elapsed


        Channel[timer] = -1; // this will get incremented at the end of the refresh period to start again at the first channel

    }
}

#ifndef WIRING // Wiring pre-defines signal handlers so don't define any if compiling for the Wiring platform
// Interrupt handlers for Arduino
#if defined(_useTimer1)
ISR(TIMER1_COMPA_vect)
{
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}
#endif
#if defined(_useTimer3)
ISR(TIMER3_COMPA_vect)
{
    handle_interrupts(_timer3, &TCNT3, &OCR3A);
}
#endif

#if defined(_useTimer4)
ISR(TIMER4_COMPA_vect)
{
    handle_interrupts(_timer4, &TCNT4, &OCR4A);
}
#endif

#if defined(_useTimer5)
ISR(TIMER5_COMPA_vect)
{
    handle_interrupts(_timer5, &TCNT5, &OCR5A);
}
#endif

#elif defined WIRING
// Interrupt handlers for Wiring
#if defined(_useTimer1)
void Timer1Service()
{
    handle_interrupts(_timer1, &TCNT1, &OCR1A);
}
#endif
#if defined(_useTimer3)
void Timer3Service()
{
    handle_interrupts(_timer3, &TCNT3, &OCR3A);
}
#endif
#endif


static void initISR(timer16_Sequence_t timer)
{
#if defined (_useTimer1)
    if(timer == _timer1) {
        TCCR1A = 0;             // normal counting mode
        TCCR1B = _BV(CS11);     // set prescaler of 8
        TCNT1 = 0;              // clear the timer count
#if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIFR |= _BV(OCF1A);      // clear any pending interrupts;
        TIMSK |=  _BV(OCIE1A) ;  // enable the output compare interrupt
#else
        // here if not ATmega8 or ATmega128
        TIFR1 |= _BV(OCF1A);     // clear any pending interrupts;
        TIMSK1 |=  _BV(OCIE1A) ; // enable the output compare interrupt
#endif
#if defined(WIRING)
        timerAttach(TIMER1OUTCOMPAREA_INT, Timer1Service);
#endif
    }
#endif


#if defined (_useTimer3)
    if(timer == _timer3) {
        TCCR3A = 0;             // normal counting mode
        TCCR3B = _BV(CS31);     // set prescaler of 8
        TCNT3 = 0;              // clear the timer count
#if defined(__AVR_ATmega128__)
        TIFR |= _BV(OCF3A);     // clear any pending interrupts;
        ETIMSK |= _BV(OCIE3A);  // enable the output compare interrupt
#else
        TIFR3 = _BV(OCF3A);     // clear any pending interrupts;
        TIMSK3 =  _BV(OCIE3A) ; // enable the output compare interrupt
#endif
#if defined(WIRING)
        timerAttach(TIMER3OUTCOMPAREA_INT, Timer3Service);  // for Wiring platform only
#endif
    }
#endif

#if defined (_useTimer4)
    if(timer == _timer4) {
        TCCR4A = 0;             // normal counting mode
        TCCR4B = _BV(CS41);     // set prescaler of 8
        TCNT4 = 0;              // clear the timer count
        TIFR4 = _BV(OCF4A);     // clear any pending interrupts;
        TIMSK4 =  _BV(OCIE4A) ; // enable the output compare interrupt
    }
#endif

#if defined (_useTimer5)
    if(timer == _timer5) {
        TCCR5A = 0;             // normal counting mode
        TCCR5B = _BV(CS51);     // set prescaler of 8
        TCNT5 = 0;              // clear the timer count
        TIFR5 = _BV(OCF5A);     // clear any pending interrupts;
        TIMSK5 =  _BV(OCIE5A) ; // enable the output compare interrupt
    }
#endif
}

/****************** end of static functions ******************************/
MeServo::MeServo(): MePort(0){
	
}
MeServo::MeServo(uint8_t port, uint8_t device): MePort(port)
{
    servoPin = ( device == DEV1 ? s2 : s1);
    reset(port,device);
}
void MeServo::reset(uint8_t port, uint8_t device)
{
	MePort::reset(port, device);
    servoPin = ( device == DEV1 ? s2 : s1);
    if(port>0){
	    if( ServoCount < MAX_SERVOS) {
	        this->servoIndex = ServoCount++;                    // assign a servo index to this instance
	        servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values  - 12 Aug 2009
	    } else
	        this->servoIndex = INVALID_SERVO ;  // too many servos
    }
}
uint8_t MeServo::begin()
{
    return this->begin(MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t MeServo::begin(int min, int max)
{
    if(this->servoIndex < MAX_SERVOS ) {
        pinMode( servoPin, OUTPUT) ;                                   // set servo pin to output
        servos[this->servoIndex].Pin.nbr = servoPin;
        // todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
        this->min  = (MIN_PULSE_WIDTH - min) / 4; //resolution of min/max is 4 uS
        this->max  = (MAX_PULSE_WIDTH - max) / 4;
        // initialize the timer if it has not already been initialized
        timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
        if(isTimerActive(timer) == false)
            initISR(timer);
        servos[this->servoIndex].Pin.isActive = true;  // this must be set after the check for isTimerActive
    }
    return this->servoIndex ;
}

void MeServo::detach()
{
    servos[this->servoIndex].Pin.isActive = false;
    timer16_Sequence_t timer = SERVO_INDEX_TO_TIMER(servoIndex);
    if(isTimerActive(timer) == false) {
        finISR(timer);
    }
	ServoCount--;
}

void MeServo::write(int value)
{
    int delayTime = abs(value - this->read());
    this->begin();
    if(value < MIN_PULSE_WIDTH) {
        // treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
        if(value < 0) value = 0;
        if(value > 180) value = 180;
        value = map(value, 0, 180, SERVO_MIN(),  SERVO_MAX());
    }
    this->writeMicroseconds(value);
    //delay(delayTime);
    //this->detach();
}

void MeServo::writeMicroseconds(int value)
{
    // calculate and store the values for the given channel
    byte channel = this->servoIndex;
    if( (channel < MAX_SERVOS) ) { // ensure channel is valid
        if( value < SERVO_MIN() )          // ensure pulse width is valid
            value = SERVO_MIN();
        else if( value > SERVO_MAX() )
            value = SERVO_MAX();

        value = value - TRIM_DURATION;
        value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead - 12 Aug 2009

        uint8_t oldSREG = SREG;
        cli();
        servos[channel].ticks = value;
        SREG = oldSREG;
    }
}

int MeServo::read()   // return the value as degrees
{
    return  map( this->readMicroseconds() + 1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int MeServo::readMicroseconds()
{
    unsigned int pulsewidth;
    if( this->servoIndex != INVALID_SERVO )
        pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION ;   // 12 aug 2009
    else
        pulsewidth  = 0;

    return pulsewidth;
}

bool MeServo::attached()
{
    return servos[this->servoIndex].Pin.isActive ;
}


/*Me4Button*/
Me4Button::Me4Button() : MePort(0){
	
}
Me4Button::Me4Button(uint8_t port) : MePort(port)
{
    _toggleState = NULL_KEY;
    _oldState = NULL_KEY_VALUE;
	_prevPressedState = 0;
    _pressedState = NULL_KEY;
    _releasedState = NULL_KEY;
    _heldState = NULL_KEY;
    _heldTime = millis();
}

bool Me4Button::update()
{
	if(millis()-_heldTime<10){
		return false;
	}
    uint8_t update_temp;
    uint16_t newState = 0;
	uint16_t t=0;
	uint16_t tmax = 0;
    for(uint8_t i = 0; i < 16; i++) {
		t = MePort::Aread2();
		if(i<4){
			continue;
		}
        newState += t;
		if(tmax<t){
			tmax = t;
		}
    }
	newState -=tmax;
    newState /= 11;
    if(abs(newState-_oldState) > 40)update_temp = 1;
	else update_temp = 0;	
    if (update_temp) {
		if(newState > KEY4_VALUE+50) { //released?
	            _toggleState = !_toggleState;
	            if(_oldState < KEY1_VALUE+5)_releasedState = KEY1;
	            else if(_oldState > KEY2_VALUE - 5 && _oldState < KEY2_VALUE + 5)_releasedState = KEY2;
	            else if(_oldState > KEY3_VALUE - 5 && _oldState < KEY3_VALUE + 5)_releasedState = KEY3;
	            else if(_oldState > KEY4_VALUE - 5 && _oldState < KEY4_VALUE + 5)_releasedState = KEY4;
				

				_pressedState = NULL_KEY;
	    }else{
	        if(newState < KEY1_VALUE+5)_pressedState = KEY1;
	        else if((newState > KEY2_VALUE - 5) && (newState < KEY2_VALUE + 5))_pressedState = KEY2;
	        else if((newState > KEY3_VALUE - 5) && (newState < KEY3_VALUE + 5))_pressedState = KEY3;
	        else if((newState > KEY4_VALUE - 5) && (newState < KEY4_VALUE + 5))_pressedState = KEY4;	
		}
        //delay(10); // debouncing
    } else {
       	if(_oldState < (KEY4_VALUE + 10))_pressedState = NULL_KEY;
		if(newState > KEY4_VALUE+5 && _oldState > KEY4_VALUE+5){
			_releasedState = NULL_KEY;
		}
    }
    _oldState = newState;
	_heldTime = millis();
	return true;
}


uint8_t Me4Button::pressed()
{
	update();
	uint8_t returnKey = _pressedState;
	_pressedState = NULL_KEY;
	return returnKey;
}

uint8_t Me4Button::released()
{
	update();
	uint8_t returnKey = _releasedState;
	_releasedState = NULL_KEY;
	return returnKey;
}

/*      Joystick        */
MeJoystick::MeJoystick() : MePort(0){}
MeJoystick::MeJoystick(uint8_t port) : MePort(port){}

int MeJoystick::readX()
{	
	int mapX = map(MePort::Aread1(),1,980,-255,255);
	return abs(mapX)<30?0:mapX ;
}

int MeJoystick::readY()
{
    
    int mapY = map(MePort::Aread2(),24,980,-255,255);
	return abs(mapY)<30?0:mapY ;
}

float MeJoystick::angle(){
	return atan2(readY(),readX())*180.0/PI;
}

float MeJoystick::strength(){
	long dx = abs(readX());
	long dy = abs(readY());
	long dist = dx*dx+dy*dy;
	return min(1.0,sqrt(dist)/255.0);
}

/*      Light Sensor        */
MeLightSensor::MeLightSensor() : MePort(0){}
MeLightSensor::MeLightSensor(uint8_t port) : MePort(port){}
int MeLightSensor::read()
{	
	return MePort::Aread2();
}

void MeLightSensor::lightOn()
{	
	MePort::Dwrite1(HIGH);
}

void MeLightSensor::lightOff()
{	
	MePort::Dwrite1(LOW);
}

float MeLightSensor::strength()
{
    
    return map(MePort::Aread2(),0,1023,0,1023);
}

/*      Sound Sensor        */
MeSoundSensor::MeSoundSensor() : MePort(0){}
MeSoundSensor::MeSoundSensor(uint8_t port) : MePort(port){}
bool MeSoundSensor::Dread()
{	
	return MePort::Dread1();
}

int MeSoundSensor::strength()
{
    
    return MePort::Aread2();
}
OneWire::OneWire(){
	
}
OneWire::OneWire(uint8_t pin)
{
	pinMode(pin, INPUT_PULLUP);
	bitmask = PIN_TO_BITMASK(pin);
	baseReg = PIN_TO_BASEREG(pin);
	reset_search();
}
void OneWire::reset(uint8_t pin)
{
	pinMode(pin, INPUT_PULLUP);
	bitmask = PIN_TO_BITMASK(pin);
	baseReg = PIN_TO_BASEREG(pin);
	reset_search();
}

// Perform the onewire reset function.  We will wait up to 250uS for
// the bus to come high, if it doesn't then it is broken or shorted
// and we return a 0;
//
// Returns 1 if a device asserted a presence pulse, 0 otherwise.
//
uint8_t OneWire::reset(void)
{
	IO_REG_TYPE mask = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	uint8_t r;
	uint8_t retries = 125;

	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);
	interrupts();
	// wait until the wire is high... just in case
	do {
		if (--retries == 0) return 0;
		delayMicroseconds(2);
	} while ( !DIRECT_READ(reg, mask));

	noInterrupts();
	DIRECT_WRITE_LOW(reg, mask);
	DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
	interrupts();
	delayMicroseconds(480);
	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(70);
	r = !DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(410);
	return r;
}

//
// Write a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
void OneWire::write_bit(uint8_t v)
{
	IO_REG_TYPE mask=bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;

	if (v & 1) {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(10);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(55);
	} else {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(65);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(5);
	}
}

//
// Read a bit. Port and bit is used to cut lookup time and provide
// more certain timing.
//
uint8_t OneWire::read_bit(void)
{
	IO_REG_TYPE mask=bitmask;
	volatile IO_REG_TYPE *reg IO_REG_ASM = baseReg;
	uint8_t r;

	noInterrupts();
	DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	delayMicroseconds(3);
	DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(10);
	r = DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(53);
	return r;
}

//
// Write a byte. The writing code uses the active drivers to raise the
// pin high, if you need power after the write (e.g. DS18S20 in
// parasite power mode) then set 'power' to 1, otherwise the pin will
// go tri-state at the end of the write to avoid heating in a short or
// other mishap.
//
void OneWire::write(uint8_t v, uint8_t power /* = 0 */) {
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	OneWire::write_bit( (bitMask & v)?1:0);
    }
    if ( !power) {
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	DIRECT_WRITE_LOW(baseReg, bitmask);
	interrupts();
    }
}

void OneWire::write_bytes(const uint8_t *buf, uint16_t count, bool power /* = 0 */) {
  for (uint16_t i = 0 ; i < count ; i++)
    write(buf[i]);
  if (!power) {
    noInterrupts();
    DIRECT_MODE_INPUT(baseReg, bitmask);
    DIRECT_WRITE_LOW(baseReg, bitmask);
    interrupts();
  }
}

//
// Read a byte
//
uint8_t OneWire::read() {
    uint8_t bitMask;
    uint8_t r = 0;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( OneWire::read_bit()) r |= bitMask;
    }
    return r;
}

void OneWire::read_bytes(uint8_t *buf, uint16_t count) {
  for (uint16_t i = 0 ; i < count ; i++)
    buf[i] = read();
}

//
// Do a ROM select
//
void OneWire::select(const uint8_t rom[8])
{
    uint8_t i;

    write(0x55);           // Choose ROM

    for (i = 0; i < 8; i++) write(rom[i]);
}

//
// Do a ROM skip
//
void OneWire::skip()
{
    write(0xCC);           // Skip ROM
}

void OneWire::depower()
{
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	interrupts();
}

void OneWire::reset_search()
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = FALSE;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) {
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}

// Setup the search to find the device type 'family_code' on the next call
// to search(*newAddr) if it is present.
//
void OneWire::target_search(uint8_t family_code)
{
   // set the search state to find SearchFamily type devices
   ROM_NO[0] = family_code;
   for (uint8_t i = 1; i < 8; i++)
      ROM_NO[i] = 0;
   LastDiscrepancy = 64;
   LastFamilyDiscrepancy = 0;
   LastDeviceFlag = FALSE;
}

//
// Perform a search. If this function returns a '1' then it has
// enumerated the next device and you may retrieve the ROM from the
// OneWire::address variable. If there are no devices, no further
// devices, or something horrible happens in the middle of the
// enumeration then a 0 is returned.  If a new device is found then
// its address is copied to newAddr.  Use OneWire::reset_search() to
// start over.
//
// --- Replaced by the one from the Dallas Semiconductor web site ---
//--------------------------------------------------------------------------
// Perform the 1-Wire Search Algorithm on the 1-Wire bus using the existing
// search state.
// Return TRUE  : device found, ROM number in ROM_NO buffer
//        FALSE : device not found, end of search
//
uint8_t OneWire::search(uint8_t *newAddr)
{
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number, search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = 0;

   // if the last call was not the last one
   if (!LastDeviceFlag)
   {
      // 1-Wire reset
      if (!reset())
      {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = FALSE;
         LastFamilyDiscrepancy = 0;
         return FALSE;
      }

      // issue the search command
      write(0xF0);

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = read_bit();
         cmp_id_bit = read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1))
            break;
         else
         {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit)
               search_direction = id_bit;  // bit write value for search
            else
            {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy)
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               else
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);

               // if 0 was picked then record its position in LastZero
               if (search_direction == 0)
               {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            write_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0)
            {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65))
      {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0)
            LastDeviceFlag = TRUE;

         search_result = TRUE;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0])
   {
      LastDiscrepancy = 0;
      LastDeviceFlag = FALSE;
      LastFamilyDiscrepancy = 0;
      search_result = FALSE;
   }
   for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   return search_result;
  }
MeTemperature::MeTemperature():MePort(){
	
}
MeTemperature::MeTemperature(uint8_t port):MePort(port){
	_ts.reset(s2);
}
MeTemperature::MeTemperature(uint8_t port,uint8_t slot):MePort(port){
	MePort::reset(port, slot);
    _ts.reset( slot == DEV1 ? s2 : s1);
}
void MeTemperature::reset(uint8_t port,uint8_t slot){
	MePort::reset(port, slot);
    _ts.reset( slot == DEV1 ? s2 : s1);
}
float MeTemperature::temperature(){
	byte i;
	byte present = 0;
	byte type_s;
	byte data[12];
	byte addr[8];
	float celsius;
	if ( !_ts.search(addr)) {
	_ts.reset_search();
	return -100;
	}
	
	_ts.reset();
	_ts.select(addr);
	_ts.write(0x44, 1);        // start conversion, with parasite power on at the end
	present = _ts.reset();
	_ts.select(addr);    
	_ts.write(0xBE);         
	for ( i = 0; i < 9; i++) {           // we need 9 bytes
	data[i] = _ts.read();
	}
	int16_t raw = (data[1] << 8) | data[0];
	byte cfg = (data[4] & 0x60);
	if (cfg == 0x00) raw = raw & ~7;
	else if (cfg == 0x20) raw = raw & ~3; 
	else if (cfg == 0x40) raw = raw & ~1; 
	
	celsius = (float)raw / 16.0;
	return celsius;
}

static int8_t TubeTab[] = {0x3f,0x06,0x5b,0x4f,
                           0x66,0x6d,0x7d,0x07,
                           0x7f,0x6f,0x77,0x7c,
                           0x39,0x5e,0x79,0x71,
						   0xbf,0x86,0xdb,0xcf,
						   0xe6,0xed,0xfd,0x87,
						   0xff,0xef,0xf7,0xfc,
						   0xb9,0xde,0xf9,0xf1,0x40};//0~9,A,b,C,d,E,F,-                    
MeNumericDisplay::MeNumericDisplay():MePort()
{
}
MeNumericDisplay::MeNumericDisplay(uint8_t port):MePort(port)
{
  Clkpin = s2;
  Datapin = s1;
  pinMode(Clkpin,OUTPUT);
  pinMode(Datapin,OUTPUT);
  set();
  clearDisplay();
}
void MeNumericDisplay::reset(uint8_t port){
  reset(port);
  Clkpin = s2;
  Datapin = s1;
  pinMode(Clkpin,OUTPUT);
  pinMode(Datapin,OUTPUT);
  set();
  clearDisplay();
}
void MeNumericDisplay::init(void)
{
  clearDisplay();
}

void MeNumericDisplay::writeByte(int8_t wr_data)
{
  uint8_t i,count1;   
  for(i=0;i<8;i++)        //sent 8bit data
  {
    digitalWrite(Clkpin,LOW);      
    if(wr_data & 0x01)digitalWrite(Datapin,HIGH);//LSB first
    else digitalWrite(Datapin,LOW);
    wr_data >>= 1;      
    digitalWrite(Clkpin,HIGH);
      
  }  
  digitalWrite(Clkpin,LOW); //wait for the ACK
  digitalWrite(Datapin,HIGH);
  digitalWrite(Clkpin,HIGH);     
  pinMode(Datapin,INPUT);
  while(digitalRead(Datapin))    
  { 
    count1 +=1;
    if(count1 == 200)//
    {
     pinMode(Datapin,OUTPUT);
     digitalWrite(Datapin,LOW);
     count1 =0;
    }
    //pinMode(Datapin,INPUT);
  }
  pinMode(Datapin,OUTPUT);
  
}
//send start signal to TM1637
void MeNumericDisplay::start(void)
{
  digitalWrite(Clkpin,HIGH);//send start signal to TM1637
  digitalWrite(Datapin,HIGH); 
  digitalWrite(Datapin,LOW); 
  digitalWrite(Clkpin,LOW); 
} 
//End of transmission
void MeNumericDisplay::stop(void)
{
  digitalWrite(Clkpin,LOW);
  digitalWrite(Datapin,LOW);
  digitalWrite(Clkpin,HIGH);
  digitalWrite(Datapin,HIGH); 
}
void MeNumericDisplay::display(float value){
	
	int i=0;
	bool isStart = false;
	int index = 0;
	int8_t disp[]={
		0,0,0,0
	};
	bool isNeg = false;
	if(value<0){
		isNeg = true;
		value = -value;
		disp[0] = 0x20;
		index++;
	}
	for(i=0;i<7;i++){
		int n = checkNum(value,3-i);
		if(n>=1||i==3){
		 isStart=true; 
		}
		if(isStart){
		  if(i==3){
	 		disp[index]=n+0x10;
		  }else{
		 	disp[index]=n;
		  }
		  index++;
	  	}
		if(index>3){
		 break; 
		}
	}
	display(disp);
}
int MeNumericDisplay::checkNum(float v,int b){
 if(b>=0){
	return floor((v-floor(v/pow(10,b+1))*(pow(10,b+1)))/pow(10,b));
 }else{
	b=-b;
	int i=0;
 	for(i=0;i<b;i++){
  		v = v*10;
  	}
	return ((int)(v)%10);
 }
}

void MeNumericDisplay::display(int8_t DispData[])
{
  int8_t SegData[4];
  uint8_t i;
  for(i = 0;i < 4;i ++)
  {
    SegData[i] = DispData[i];
  }
  coding(SegData);
  start();          //start signal sent to TM1637 from MCU
  writeByte(ADDR_AUTO);//
  stop();           //
  start();          //
  writeByte(Cmd_SetAddr);//
  for(i=0;i < 4;i ++)
  {
    writeByte(SegData[i]);        //
  }
  stop();           //
  start();          //
  writeByte(Cmd_DispCtrl);//
  stop();           //
}
//******************************************
void MeNumericDisplay::display(uint8_t BitAddr,int8_t DispData)
{
  int8_t SegData;
  SegData = coding(DispData);
  start();          //start signal sent to TM1637 from MCU
  writeByte(ADDR_FIXED);//
  stop();           //
  start();          //
  writeByte(BitAddr|0xc0);//
  writeByte(SegData);//
  stop();            //
  start();          //
  writeByte(Cmd_DispCtrl);//
  stop();           //
}

void MeNumericDisplay::clearDisplay(void)
{
  display(0x00,0x7f);
  display(0x01,0x7f);
  display(0x02,0x7f);
  display(0x03,0x7f);  
}
//To take effect the next time it displays.
void MeNumericDisplay::set(uint8_t brightness,uint8_t SetData,uint8_t SetAddr)
{
  Cmd_SetData = SetData;
  Cmd_SetAddr = SetAddr;
  Cmd_DispCtrl = 0x88 + brightness;//Set the brightness and it takes effect the next time it displays.
}


void MeNumericDisplay::coding(int8_t DispData[])
{
  uint8_t PointData = 0; 
  for(uint8_t i = 0;i < 4;i ++)
  {
    if(DispData[i] == 0x7f)DispData[i] = 0x00;
    else DispData[i] = TubeTab[DispData[i]];
  }
}
int8_t MeNumericDisplay::coding(int8_t DispData)
{
  uint8_t PointData = 0; 
  if(DispData == 0x7f) DispData = 0x00 + PointData;//The bit digital tube off
  else DispData = TubeTab[DispData] + PointData;
  return DispData;
}
MePotentiometer::MePotentiometer():MePort(0){

}
MePotentiometer::MePotentiometer(uint8_t port):MePort(port){

}
uint16_t MePotentiometer::read(){
	return MePort::Aread2();
}
/***********Me GYRO*********/
MeGyro::MeGyro():MePort(0){

}
void MeGyro::begin(){
	close();
	gSensitivity = 65.5; // for 500 deg/s, check data sheet
	gx = 0;
	gy = 0;
	gz = 0;
	gyrX = 0;
	gyrY = 0;
	gyrZ = 0;
	accX = 0;
	accY = 0;
	accZ = 0;
	gyrXoffs = -281.00;
	gyrYoffs = 18.00;
	gyrZoffs = -83.00;
	FREQ=30.0;
	if(!isWireBegin){
		isWireBegin = true;
	}
	Wire.begin();
	delay(1000);
	int error = writeReg (0x6b, 0x00);
	error += writeReg (0x1a, 0x01);
	error += writeReg(0x1b, 0x08);
	uint8_t sample_div = 1000 / FREQ - 1;
	error +=writeReg (0x19, sample_div);
}
double MeGyro::angleX(){
	if(accZ==0)return 0;
	return (atan((float)accX/(float)accZ))*180/3.1415926;
} 
double MeGyro::angleY(){
	if(accZ==0)return 0;
	return (atan((float)accY/(float)accZ))*180/3.1415926;
} 
double MeGyro::angleZ(){
	if(gyrZ==0)return 0;
	return gyrZ/10.0;
} 
void MeGyro::close(){
}
void MeGyro::update(){
	 unsigned long start_time = millis();
  uint8_t error;
  // read imu data
  error = readData(0x3b, i2cData, 14);
  if(error!=0)
    return;

  double ax, ay, az;
  // assemble 16 bit sensor data
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);

  gyrX = (((i2cData[8] << 8) | i2cData[9]) - gyrXoffs) / gSensitivity;
  gyrY = (((i2cData[10] << 8) | i2cData[11]) - gyrYoffs) / gSensitivity;
  gyrZ = (((i2cData[12] << 8) | i2cData[13]) - gyrZoffs) / (gSensitivity+1);
  ay = atan2(accX, sqrt( pow(accY, 2) + pow(accZ, 2))) * 180 / M_PI;
  ax = atan2(accY, sqrt( pow(accX, 2) + pow(accZ, 2))) * 180 / M_PI;

  // angles based on gyro (deg/s)
  gx = gx + gyrX / FREQ;
  gy = gy - gyrY / FREQ;
  gz += gyrZ / FREQ;

  // complementary filter
  // tau = DT*(A)/(1-A)
  // = 0.48sec
  gx = gx * 0.96 + ax * 0.04;
  gy = gy * 0.96 + ay * 0.04;

  //delay(((1/FREQ) * 1000) - (millis() - start_time)-time);
}
int MeGyro::readData(int start, uint8_t *buffer, int size)
{
	int i, n, error;
	Wire.beginTransmission(0x68);
	n = Wire.write(start);
	if (n != 1)
	return (-10);
	n = Wire.endTransmission(false);    // hold the I2C-bus
	if (n != 0)
	return (n);
	delayMicroseconds(1);
	// Third parameter is true: relase I2C-bus after data is read.
	Wire.requestFrom(0x68, size, true);
	i = 0;
	while(Wire.available() && i<size)
	{
	buffer[i++]=Wire.read();
	}
	delayMicroseconds(1);
	if ( i != size)
	return (-11);
	return (0);  // return : no error
}
int MeGyro::writeData(int start, const uint8_t *pData, int size)
{
  int n, error;
  Wire.beginTransmission(0x68);
  n = Wire.write(start);        // write the start address
  if (n != 1)
  return (-20);
  n = Wire.write(pData, size);  // write data bytes
  if (n != size)
  return (-21);
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
  return (error);
  return (0);         // return : no error
}
int MeGyro::writeReg(int reg, uint8_t data)
{
  int error;
  error = writeData(reg, &data, 1);
  return (error);
}
