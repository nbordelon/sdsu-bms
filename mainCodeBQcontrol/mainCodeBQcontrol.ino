#include <Wire.h>

//Initialize Global Variables

int bqAddr = 0x08; //BQ76940 Uses the address 0x08 with its I2C communications
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
  
  Serial.begin(9600); //Baud Rate 9600
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
    Serial.print(": ");
    cellVolts[i-1] = readVoltages(i);
    Serial.println(cellVolts[i-1]);
  }

  //Read the pack voltage and add it to a global float variable packVolt

  Serial.print("Pack Voltage: ");
  Serial.println(readPackVoltage());
  packVolt = readPackVoltage();

  if(ISR_triggered)
  {
    Serial.println("ISR TRIGGERED, ALERT PIN HIGH");
    Serial.print("0x");
    Serial.println(readRegister(0x0),HEX); //Shows the System Status register in hex

    //ALERT PIN HANDLING
    //CHECK EACH SYSTEM STATUS BIT AND HANDLE ACCORDINGLY


    ISR_triggered = false;
  }
  
  digitalWrite(statusLED, LOW);
  delay(1000);
}

//Ran in the setup block, Initializes pins for correct run mode 
//and initializes global variables

boolean initializeBQ(byte inrptPin)
{
  //Enable Init Bits ADC_EN and CC_EN and CC_CFG

  writeRegister(0x0B,0x19); //Data sheet specifies CC_CFG should be written 0x19

  byte sys_ctrl1 = readRegister(0x04); //Enableing ADC_EN
  sys_ctrl1 |= 1 << 4;
  writeRegister(0x04,sys_ctrl1);
  
  byte sys_ctrl2 = readRegister(0x05);
  sys_ctrl2 |= 1 << 6;
  writeRegister(0x05,sys_ctrl2);
  
  //Initialize Gain and Offset used to calculate voltage from ADC reading
  
  gain = readGain();
  offset = readOffset();

  //Initialize interrupt on pin 2 connected to the ALERT pin from the BQ

  pinMode(2, INPUT);
  attachInterrupt(0, alertPinISR, RISING);

  return true;
}

//Read a single register, a byte / 8 bits, specified address in the argument

byte readRegister(byte addrRegister)
{
  Wire.beginTransmission(bqAddr);
  Wire.write(addrRegister);
  Wire.endTransmission();

  Wire.requestFrom(bqAddr, 1);

  return (Wire.read());
}

//Read two registers next to each other starting with the first one in succession.
//Return the two registers concatenated together into a 16 bit integer variable.

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

//Write a byte of data into a register through I2C

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

//Read the gain given by the BQ76940 registers ADCgain1 and ADCgain2
//Page 40 of data sheet GAIN = 365 uV/LSB + (ADCGAIN<4:0>in decimal) x (1 uV/LSB)

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

//Read the pack voltage from the BAT_hi and BAT_low registers and return as a float

float readPackVoltage()
{
  unsigned int adcPackVolt = doubleReadRegister(0x2A);

  float packVoltage = 4 * gain * adcPackVolt;
  packVoltage += 4 * offset;

  return(packVoltage / (float)1000);
}

//Enable Cell Balancing on a specific cell number should be 1,2,3,4
//4 is actually 5. 4 doesn't access a cell but I keep 4 as an input for simplicity.

byte enableCellBalance(byte cellNum, boolean en)
{
  if(cellNum < 1 || cellNum > 4) return 0;
  if(cellNum == 4) cellNum++;
  
  byte CELLBAL1 = readRegister(0x01);
  CELLBAL1 |= 1 << (cellNum - 1);
  writeRegister(0x01,CELLBAL1);

  return 0;
}

//Read CoulombCounter Register and return a float value

float readCC(void)
{
  int count = doubleReadRegister(0x32);

  float count_uV = count * 8.44;

  return(count_uV);
}

//read the temp from the chips, 0 is the internal die temp and 1-3 are thermistors
//WIP

int readTemp(byte thermistorNum)
{
  if(thermistorNum < 0 || thermistorNum > 3) return(0);
  
}
