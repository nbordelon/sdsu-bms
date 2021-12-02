#include <Wire.h>
#include <SoftwareSerial.h>

//https://github.com/nseidle/BMS/blob/master/firmware/SparkFun_bq769x0/SparkFun_bq769x0.ino
//////////////////////////////////////////////////////////////////////////////
////////////////////Initialize Global Variables///////////////////////////////

int bqAddr = 0x18; //BQ76920 Uses the address 0x08 with its I2C communications
float gain = 0;
int offset = 0;
byte statusLED = 13;
float cellVolts[4] = {0.0,0.0,0.0,0.0};
float packVolt = 0.0;
float cCount = 0.0;
float prevcCount = 0.0;
float temp = 0.0;
volatile boolean ISR_triggered = false;
long timeLoop;
long timeLoop2;
long ledLoop;
boolean ccReady = false;
boolean balFlag = false;
boolean balDone = false;
SoftwareSerial OpenLCD(0, 1); //RX, TX
byte counter = 0;
byte contrast = 4;
int buttonS = 0;
int buttonR = 0;
byte incrementB = 0;
float outCurrent = 0.0;
boolean resetS = false;
float chargeP = 0.0;

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//////Interrupt Service Routine to handle an Alert pin input from the BQ//////

void alertPinISR()
{
  ISR_triggered = true;
}



//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//////Runs once at the beginning of the code//////////////////////////////////

void setup() {
  OpenLCD.begin(9600); //Start communication with OpenLCD

  //Send contrast setting
  OpenLCD.write('|'); //Put LCD into setting mode
  OpenLCD.write(24); //Send contrast command
  OpenLCD.write(contrast);
  OpenLCD.write('|'); //Setting character
  OpenLCD.write('-'); //Clear display
  Wire.begin(); //Start I2C Comms
  
  OpenLCD.print("I2C start");

  if(initializeBQ(2) == false)
  {
    OpenLCD.print("BQ Init Failed. Hanging......");
    while(1);
  }
  else
  {
    OpenLCD.print("BQ Init Success");
  }

  //Clear System Status Register on Reset
  writeRegister(0x00,0b11111111);

  enCHGfet(true);
  enDSGfet(true);
  
  timeLoop = millis();
  timeLoop2 = millis();
  ledLoop = millis();
  
  //writeRegister(0x06,0b10000111);

  pinMode(3,INPUT); //reset Switch
  pinMode(4,INPUT); //button
  pinMode(5, OUTPUT);//red1
  pinMode(6, OUTPUT);//green2
  pinMode(7, OUTPUT);//green3  
  pinMode(8, OUTPUT);//green4    
  pinMode(9, OUTPUT);//green5
  pinMode(10,OUTPUT);//statusLED

}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//////Continuosly loops through out runtime///////////////////////////////////

void loop() {
  
  //Handling Button to adjust LCD Screen
    buttonS = digitalRead(4);
    buttonR = digitalRead(3);

    if(buttonS == LOW)
    {
      if(incrementB < 2)
      {
        incrementB++; 
      }
      else
      {
        incrementB = 0;
      }
    }
    else{

    }

    if(outCurrent > 15.00)
    {
      enDSGfet(false);
    }
    else
    {
      enDSGfet(true);
    }

    if(temp > 30.0)
    {
      enCHGfet(false);
      enDSGfet(false);    
    }

    if(buttonR == LOW)
    {
      writeRegister(0x0,0xFF);

      cCount = 0;
      outCurrent = 0;
      digitalWrite(10,HIGH);
      resetS = true;  
    }
    else
    {
      digitalWrite(10,LOW);
      resetS = false;
    }




  if(millis() - timeLoop > 1000)
  {

    //read individual cell voltages
    for(byte i = 1; i < 5 ; i ++)
    {
      cellVolts[i-1] = readVoltages(i);
    }

    //Read the pack voltage and add it to a global float variable packVolt
    
    packVolt = readPackVoltage();

    
    chargeP = (14.6 - packVolt) / 4.6;
    if(chargeP < 0) chargeP = 0;

    //Read the temperature and set it in a global variable.

    temp = readTemp(0);

    if(prevcCount != 0.0 && cCount != 0.0)
    {
      outCurrent = (cCount - prevcCount+ 220) / 420;
    }

//    OpenLCD.print(prevcCount);
//    OpenLCD.print(" ");
//    OpenLCD.print(cCount);
//    OpenLCD.print(" "); 
    prevcCount = cCount;
    
    timeLoop = millis();

    lcdScreenOutput(incrementB + 1);

    chargePercentage(chargeP);
  }

  if(ISR_triggered == true )
  {

    byte sysStatus = readRegister(0x0);
    
    //ALERT PIN HANDLING
    //CHECK EACH SYSTEM STATUS BIT AND HANDLE ACCORDINGLY
    byte newSystemStatus = 0;
   
    if(sysStatus & (1<<7)) //CC_READY BIT
    {
      //Read Coulomb Counter Register and return a float to serial monitor
      cCount += readCC();
      newSystemStatus |= (1<<7);
    }

    if(sysStatus & (1<<5)) //DEVICE_XREADY
    {
      newSystemStatus |= (1<<5);
    }

    if(sysStatus & (1<<4)) //OVRD_ALERT
    {
      newSystemStatus |= (1<<4);
    }

    if(sysStatus & (1<<3)) //UV
    {
      enDSGfet(true);
      OpenLCD.write('|'); //Setting character
      OpenLCD.write('-'); //Clear display
      OpenLCD.print("UV Alert Need Op");
      OpenLCD.print("Reset System.");
      while(!resetS);
      newSystemStatus |= (1<<3);
    }

    if(sysStatus & (1<<2)) //OV
    {
      enCHGfet(true);
      OpenLCD.write('|'); //Setting character
      OpenLCD.write('-'); //Clear display
      OpenLCD.print("OV Alert Need Op");
      OpenLCD.print("Reset System.");
      while(!resetS);
      newSystemStatus |= (1<<2);
    }

    if(sysStatus & (1<<1)) //SCD
    {
      enDSGfet(true);
      OpenLCD.write('|'); //Setting character
      OpenLCD.write('-'); //Clear display
      OpenLCD.print("SCD Alert Need   ");
      OpenLCD.print("Op Reset System.");
      while(!resetS);
      newSystemStatus |= (1<<1);
    }

    if(sysStatus & (1)) //OCD
    {
      enDSGfet(true);
      OpenLCD.write('|'); //Setting character
      OpenLCD.write('-'); //Clear display
      OpenLCD.print("OCD Alert Need   ");
      OpenLCD.print("Op Reset System.");
      while(!resetS);
      newSystemStatus |= (1);
    }

    writeRegister(0x0,newSystemStatus);
    
    ISR_triggered = false;
  }
  
  if(millis() - timeLoop2 > 60000) //Every 60 seconds check for balancing off or on
  {
    balDone = true;
    
    if(cCount > 0) //CHARGING
    {
      for(int j = 0; j < 4; j++)
      {
        for(int k = 0; k < 4; k++)
        {
          if(j != k)
          {
            if(!(-0.05 < (cellVolts[j] - cellVolts[k]) < 0.05))
            {
              for(int h = 1; h < 5; h++) 
                {
                  enableCellBalance(h,true);
                  balFlag = true;
      //            Serial.println("BALANCING ON");
                }
                break;
            }
          }
          if(balFlag) break;
        }
        if(balFlag) break;
      }
    }
    else //NOT CHARGING
    {
      for(int o = 1; o < 5; o++) 
         {
           enableCellBalance(o,false);
        //   Serial.println("BALANCING OFF");
         }
    }

    if(balFlag)
    {
      for(int j2 = 0; j2 < 4; j2++)
      {
        for(int k2 = 0; k2 < 4; k2++)
        {
          if(j2 != k2)
          {
            if(!(-0.02 < (cellVolts[j2] - cellVolts[k2]) < 0.02))
            {
              balDone = false;
              
            }
          }
        }
      }
    }

    if(balDone)
    {
      for(int o2 = 1; o2 < 5; o2++) 
                 {
                    enableCellBalance(o2,false);
          //          Serial.println("BALANCING OFF");
                 }
    }
    
    timeLoop2 = millis();
  }
  delay(1000);
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
////////Ran in the setup block, Initializes pins for correct run mode/////////

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
  
  offset = readOffset();

  //Initialize interrupt on pin 2 connected to the ALERT pin from the BQ

  pinMode(2, INPUT);
  attachInterrupt(0, alertPinISR, RISING);

  //Initializing Overvoltage and UnderVoltage Threshholds

  initOVUV(3.80,2.80);

  initSCDOCD();

  return(true);
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//Read a single register, a byte / 8 bits, specified address in the argument//

void enDSGfet(boolean en)
{
  if(en){
    byte tempBB = readRegister(0x05);
    tempBB |= (0b10);
    writeRegister(0x05,tempBB);
  }
  else
  {
    byte tempBB = readRegister(0x05);
    tempBB &= ~(0b10);
    writeRegister(0x05,tempBB);
  }
}

void enCHGfet(boolean en)
{
  if(en){
    byte tempBB = readRegister(0x05);
    tempBB |= (0b01);
    writeRegister(0x05,tempBB);
  }
  else
  {
    byte tempBB = readRegister(0x05);
    tempBB &= ~(0b01);
    writeRegister(0x05,tempBB);
  }
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

void lcdScreenOutput(byte buttonIncrement)
{
  OpenLCD.write('|'); //Setting character
  OpenLCD.write('-'); //Clear display
  switch(buttonIncrement)
  {
    case 1: //Individual Voltages
    {
      for(byte i = 1; i < 5 ; i ++)
      {
        OpenLCD.print("V");
        OpenLCD.print(i);
        OpenLCD.print(":");
        OpenLCD.print(cellVolts[i-1]);
        OpenLCD.print(" ");
      }
//          OpenLCD.print("00:");          //TESTING TAKE OUT
//          OpenLCD.print(readRegister(0x00),HEX);
//          OpenLCD.print(" 04:");
//          OpenLCD.print(readRegister(0x04),HEX);
//          OpenLCD.print("      05:");
//          OpenLCD.print(readRegister(0x05),HEX);
//          OpenLCD.print(" ");
      break;
    }
    case 2: //Pack Voltage and Temp
    {
      OpenLCD.print("Pack V: ");
      OpenLCD.print(packVolt);
      OpenLCD.print("    ");
      OpenLCD.print("Temp: ");
      OpenLCD.print(temp); 
      OpenLCD.print("      ");    
      break;
    }
    case 3: //Percent Charge
    {
      OpenLCD.print("Per CHG: ");
      OpenLCD.print(100 - (chargeP * 100));
      OpenLCD.print("% ");
      OpenLCD.print("Out Curr:");
      OpenLCD.print(outCurrent);
      OpenLCD.print("A");     
      break;
    }
    default:
    {
      break;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

byte readRegister(byte addrRegister)
{
  Wire.beginTransmission(bqAddr);
  Wire.write(addrRegister);
  Wire.endTransmission();
  Wire.requestFrom(bqAddr, 1);
  return (Wire.read());
}

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

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

float readTemp(byte thermistorNum)
{
  if(thermistorNum < 0 || thermistorNum > 3) return(0);

  byte controlReg = readRegister(0x04);
  controlReg &= ~(1<<3);
  writeRegister(0x04,controlReg);

  int thermVal = doubleReadRegister(0x2C);

  float thermVolt = thermVal * (float)382;
  thermVolt /= (float)1000000;

  float temperatureC = 25.0 - ((thermVolt - 1.2) / 0.0042);  
  return(temperatureC);
  
}

//Read and Set OV UV

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

void initSCDOCD()
{
  byte scd = readRegister(0x06);
  byte ocd = readRegister(0x07);

  scd |= 0b10000011;
  ocd |= 0b00001110;
  
  writeRegister(0x06,scd);
  writeRegister(0x07,ocd);
}

void chargePercentage(float x)
{
  x = 1 - x;
  if(0.8 <= x && x <= 1)
  {
    digitalWrite(5,HIGH); //Turn on all LEDS will be based off CHARGE PERCENT when done
    digitalWrite(6,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(8,HIGH);
    digitalWrite(9,HIGH);
  }
  else if(0.6 <= x && x < 0.8)
  {
    digitalWrite(5,HIGH); //Turn on all LEDS will be based off CHARGE PERCENT when done
    digitalWrite(6,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(8,HIGH);
    digitalWrite(9,LOW);
  }
    else if(0.4 <= x && x < 0.6)
  {
    digitalWrite(5,HIGH); //Turn on all LEDS will be based off CHARGE PERCENT when done
    digitalWrite(6,HIGH);
    digitalWrite(7,HIGH);
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
  }
    else if(0.2 <= x && x < 0.4)
  {
    digitalWrite(5,HIGH); //Turn on all LEDS will be based off CHARGE PERCENT when done
    digitalWrite(6,HIGH);
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
  }
    else if(0 <= x && x < 0.2)
  {
    digitalWrite(5,HIGH); //Turn on all LEDS will be based off CHARGE PERCENT when done
    digitalWrite(6,LOW);
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
  }
    else
    {
    digitalWrite(5,LOW); //Turn on all LEDS will be based off CHARGE PERCENT when done
    digitalWrite(6,LOW);
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
    }
  
}
