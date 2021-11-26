#include <SoftwareSerial.h>
#include <Wire.h>

byte bqAddr = 0x18;
int buttonS = 0;
byte counter = 0;
byte contrast = 4; //Lower is more contrast. 0 to 5 works for most displays.
float gain = 0;
int offset = 0;

SoftwareSerial OpenLCD(0, 1); //RX, TX

void setup() {
  // put your setup code here, to run once:

  //Serial.begin(9600);

  Wire.begin();

  Wire.setClock(100000);

  OpenLCD.begin(9600); //Start communication with OpenLCD

  //Send contrast setting
  OpenLCD.write('|'); //Put LCD into setting mode
  OpenLCD.write(24); //Send contrast command
  OpenLCD.write(contrast);

  gain = readGain() / (float)1000;

  offset = readOffset();
  
  initOVUV(3.80,2.80);
}

void loop() {
  // put your main code here, to run repeatedly:


  OpenLCD.write('|'); //Setting character
  OpenLCD.write('-'); //Clear display

  

  OpenLCD.print("Bat"); //For 16x2 LCD

  int offset = (int)readRegister(0x00);

  //writeRegister(0x09,0xFF);
  
  //Serial.println(offset);
  OpenLCD.print(counter++);

  OpenLCD.print(offset, HEX);

  

  delay(1000);
}


byte readRegister(byte addrRegister)
{
  Wire.beginTransmission(bqAddr);
  OpenLCD.print("a");
  Wire.write(addrRegister);
  OpenLCD.print("b");
  Wire.endTransmission();
  OpenLCD.print("c");
  Wire.requestFrom(bqAddr, 1);
  OpenLCD.print("d");
  return (Wire.read());
}

byte writeRegister(byte addrRegister, byte data)
{
  Wire.beginTransmission(bqAddr);
  Wire.write(addrRegister);
  Wire.endTransmission();

  Wire.beginTransmission(bqAddr);
  Wire.write(addrRegister);
  Wire.write(data);
  Wire.endTransmission();
}

void initOVUV(float over, float under){

  over *= 1000;
  under *=1000;

  over -= offset;
  over /= gain;

  under -= offset;
  under/= gain;

  int overVal = (int) over;
  int underVal = (int) under;

  overVal >>=4;
  overVal &= 0x00FF;
  underVal >>=4;
  underVal &= 0x00FF;

  writeRegister(0x09,overVal);
  writeRegister(0x0A,underVal);
  
}

int readGain()
{
  byte adcGain1 = readRegister(0x50);
  byte adcGain2 = readRegister(0x59);

  adcGain1 &= 0b00001100;

  byte adcGain = (adcGain1 << 1) | (adcGain2 >> 5);

  int gain = 365 + adcGain;
  
  return (gain);
}

int readOffset()
{
  byte offset = readRegister(0x51);

  return((int)offset);
}
