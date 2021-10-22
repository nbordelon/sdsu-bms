#include <Wire.h>
#include <SoftwareSerial.h>

//NEXT ADD SHIP MAYBE, TEMP FUNCTION, ALERT HANDLING, AND FINANLIZE INIT.
//TEST
//https://github.com/nseidle/BMS/blob/master/firmware/SparkFun_bq769x0/SparkFun_bq769x0.ino

//Initialize Global Variables

int bqAddr = 0x08; //BQ76920 Uses the address 0x08 with its I2C communications
float gain = 0;
int offset = 0;
byte statusLED = 13;
float cellVolts[4];
float packVolt = 0.0;
float cCount = 0.0;
volatile boolean ISR_triggered = false;
long timeLoop;
boolean ccReady = false;

SoftwareSerial OpenLCD(6, 7); //RX, TX
byte counter = 0;
byte contrast = 4;

//Interrupt Service Routine to handle an Alert pin input from the BQ

void alertPinISR()
{
  ISR_triggered = true;
}

//Runs once at the beginning of the code

void setup() {
  
  Serial.begin(9600); //Baud Rate 9600
  Serial.println("BQ76920 Balancing IC Init...");

  Wire.begin();

 // pinMode(statusLED,OUTPUT);
 // digitalWrite(statusLED, LOW); //Arduino LED OFF at Start

  if(initializeBQ(2) == false)
  {
    Serial.println("BQ76940 failed to respond - check your wiring");
    Serial.println("Hanging... Restart please.");
    while(1);
  }
  else
  {
    Serial.println("Initialized Successfully!");
  }

  OpenLCD.begin(9600); //Start communication with OpenLCD

  //Send contrast setting
  OpenLCD.write('|'); //Put LCD into setting mode
  OpenLCD.write(24); //Send contrast command
  OpenLCD.write(contrast);

  timeLoop = millis();
}

//Continuosly loops through out runtime

void loop() {
  // put your main code here, to run repeatedly:
    
  //digitalWrite(statusLED,HIGH);

  //Read each individual cell voltage and add them to a float array cellVolts

  if(millis() - timeLoop > 1000)
  {

    

    //read individual cell voltages
    OpenLCD.write('|'); //Setting character
    OpenLCD.write('-'); //Clear display
    for(byte i = 1; i < 5 ; i ++)
    {
      Serial.print("Cell #");
      OpenLCD.print("V");
      OpenLCD.print(i);
      OpenLCD.print(":");
      Serial.print(i);
      Serial.print(": ");
      cellVolts[i-1] = readVoltages(i);
      Serial.println(cellVolts[i-1]);
      OpenLCD.print(cellVolts[i-1]);
      OpenLCD.print(" ");
    }

    //Read the pack voltage and add it to a global float variable packVolt

    Serial.print("Pack Voltage: ");
    Serial.println(readPackVoltage());
    packVolt = readPackVoltage();

    //Read the temperature and set it in a global variable.

    float temp = readTemp(0);
    Serial.print("Temperature = ");
    Serial.println(temp);
    
    //Read Coulomb Counter Register and return a float to serial monitor
    if(ccReady)
    {
      Serial.print("Coulomb Count: ");
      Serial.println(readCC());
      cCount = readCC();
    }
    
    timeLoop = millis();

    
    OpenLCD.print("V1: "); //For 16x2 LCD
    OpenLCD.print(cellVolts[0]);
    
  }

  if(ISR_triggered == true)
  {

    byte sysStatus = readRegister(0x0);
    
    Serial.println("ISR TRIGGERED, ALERT PIN HIGH");
    Serial.print("0b");
    Serial.println(sysStatus,BIN);//Shows the System Status register in binary

    //ALERT PIN HANDLING
    //CHECK EACH SYSTEM STATUS BIT AND HANDLE ACCORDINGLY
    
    byte newSystemStatus = 0;
    
    

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

  byte sys_ctrl1 = readRegister(0x04); //Enabling ADC_EN
  sys_ctrl1 |= 1 << 4;
  writeRegister(0x04,sys_ctrl1);
  
  byte sys_ctrl2 = readRegister(0x05); //Enabling CC_EN
  sys_ctrl2 |= 1 << 6;
  writeRegister(0x05,sys_ctrl2);
  
  //Initialize Gain and Offset used to calculate voltage from ADC reading
  
  gain = readGain() / (float)1000;
  Serial.print("Gain: ");
  Serial.print(gain);
  Serial.println("mV");
  
  offset = readOffset();
  Serial.print("offset: ");
  Serial.print(offset);
  Serial.println("mV");

  //Initialize interrupt on pin 2 connected to the ALERT pin from the BQ

  pinMode(2, INPUT);
  attachInterrupt(0, alertPinISR, RISING);

  for(int l = 1; l < 5; l++)
  {
    enableCellBalance(l,true);
  }
  

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

  if(en)
  {
    CELLBAL1 |= 1 << (cellNum - 1);
    writeRegister(0x01,CELLBAL1);
  }
  else
  {
    CELLBAL1 &= ~(1 << (cellNum - 1));
    writeRegister(0x01,CELLBAL1);
  }
  
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

float readTemp(byte thermistorNum)
{
  if(thermistorNum < 0 || thermistorNum > 3) return(0);

  byte controlReg = readRegister(0x04);
  controlReg &= ~(1<<3);
  writeRegister(0x04,controlReg);

  //Serial.println("Waiting 2...");
  //delay(2000);

  int thermVal = doubleReadRegister(0x2C);

  float thermVolt = thermVal * (float)382;

  thermVolt /= (float)1000000;

  float temperatureC = 25.0 - ((thermVolt - 1.2) / 0.0042);
  
  return(temperatureC);
  
}

//Read and Set OV UV
