#include <Wire.h>

int bqAddr = 0x08; //BQ76920 Uses the address 0x08 with its I2C communications
float gain = 0;
int offset = 0;
byte statusLED = 13;
float cellVolts[4];
float packVolt = 0.0;
volatile boolean ISR_triggered = false;

//Interrupt Service Routine to handle an Alert pin input from the BQ

void alertPinISR()
{
  ISR_triggered = true;
}

//Runs once at the beginning of the code

void setup() {
  
  Serial.begin(9600);
  Serial.println("BQ76940 Balancing IC Init...");

  Wire.begin();

  pinMode(statusLED,OUTPUT);
  digitalWrite(statusLED, LOW); //Arduino LED OFF at Start

  initializeBQ(2);
}

//Continuosly loops through out runtime

void loop() {
  // put your main code here, to run repeatedly:

  digitalWrite(statusLED,HIGH);

  //Read each individual cell voltage and add them to a float array cellVolts

  for(byte i = 1; i < 5 ; i ++)
  {
    Serial.print("Cell #");
    Serial.print(i);
    Serial.print(": ")
    cellVolts[i-1] = readVoltages(i)
    Serial.println(cellVolts[i-1]);
  }

  //Read the pack voltage and add it to a global float variable packVolt

  Serial.println("Pack Voltage");
  Serial.println(readPackVoltage());
  packVolt = readPackVoltage();
  

  //while(1);
  

  digitalWrite(statusLED, LOW);
  delay(1000);
}

//Ran in the setup block, Initializes pins for correct run mode 
//and initializes global variables

boolean initializeBQ(byte inrptPin)
{
  //Enable Init Bits ADC_EN and CC_EN and CC_CFG

  
  
  //Initialize Gain and Offset used to calculate voltage from ADC reading
  
  gain = readGain();
  offset = readOffset();

  //Initialize interrupt on pin 2 connected to the ALERT pin from the BQ

  pinMode(2, INPUT);
  attachInterrupt(0, alertPinISR, RISING);
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
  Wire.beginTransmission(bqAddr);
  Wire.write(addrRegister);
  Wire.endTransmission();

  Wire.beginTransmission(bqAddr);
  Wire.write(addrRegister);
  Wire.write(data);
  Wire.endTransmission:
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

//read the temp from the chips, 0 is the internal die temp and 1-3 are thermistors

int readTemp(byte themistorNum)
{
  if(thermistorNum < 0 || thermistorNum > 3) return(0);
  
}
