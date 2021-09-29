#include <Wire.h>

int bqAddr = 0x08; //BQ76920 Uses the address 0x08 with its I2C communications
float gain = 0;
int offset = 0;
byte statusLED = 13;

void setup() {
  
  Serial.begin(9600);
  Serial.println("BQ76940 Balancing IC Init...");

  Wire.begin();

  pinMode(statusLED,OUTPUT);
  digitalWrite(statusLED, LOW); //Arduino LED OFF at Start

  initializeBQ(4);
}

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(statusLED,HIGH);

  //Serial.println("Enter Register Address to Read\n");
  //while (Serial.available() == 0){}
  //int addrInput = Serial.parseInt();

  for(byte i = 1; i < 5 ; i ++)
  {
    Serial.print("Cell: "); 
    Serial.println(readVoltages(i));
  }

  Serial.println("Pack Voltage");
  Serial.println(readPackVoltage());
  

  while(1);
  

  digitalWrite(statusLED, LOW);
  delay(1000);
}

boolean initializeBQ(byte inrptPin)
{
  gain = readGain();
  offset = readOffset();
}

byte readRegister(byte addrRegister)
{
  Wire.beginTransmission(bqAddr);
  Wire.write(addrRegister);
  Wire.endTransmission();

  Wire.requestFrom(bqAddr, 1);

  return (Wire.read());
}

int doubleReadRegister(byte addrRegister)
{
  Wire.beginTransmission(bqAddr);
  Wire.write(addrRegister);
  Wire.endTransmission();

  Wire.requestFrom(bqAddr, 2);

  byte HI = Wire.read();
  byte LO = Wire.read();

  int together  = (int)HI << 8;
  together |= LO;

  return (together);
}

byte writeRegister(byte addrRegister, byte data)
{
  
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

//Argument is 1,2,3,4. Depending on which cell you want
//Read Voltage of a individual cell by number
//The function returns a float of voltage for cell in mV

float readVoltages(byte numCell) 
{
  if(numCell < 1 || numCell > 4) return 0;

  numCell -= 1;

  if(numCell == 3) numCell += 1;
  
  int decVal = doubleReadRegister(12 + numCell * 2);

  if(decVal == 0) return 0;

  float cellVoltage = decVal * gain + offset;

  cellVoltage /= (float)1000;

  return(cellVoltage);
}

float readPackVoltage()
{
  unsigned int adcPackVolt = doubleReadRegister(0x2A);

  float packVoltage = 4 * gain * adcPackVolt;
  packVoltage += 4 * offset;

  return(packVoltage / (float)1000);
}
