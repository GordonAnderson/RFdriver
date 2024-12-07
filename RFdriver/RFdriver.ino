//
// RFdriver
//
//  This aplication is for the M0 processor on the rev 6.0 RFdriver hardware. The M0 processor 
//  is the same processor used on the adafruit feather. The Arduino IDE is used to develope this
//  application. The M0 processor emulate the SEPROM used on the MIPS modules. This RFdriver 
//  interfaces with the MIPS controller through the emulated SEPROM and the TWI interface.
//  
//  The emulated SEPROM memory configuration matches that on the previous hardware versions 
//  of the RFdriver. The interface to this module uses TWI and the address is derived from the
//  emulated SEPROM. The address set via jumpers defines the SEPROM interface and this address
//  ored with 0x20 defines the TWI command interface.
//  
//  The MIPS RFdriver firmware must use the RFdriver2, this is set in the MIPS variants.h
//  file.
//
//  The hardware design that interfaces with the RF heads is identical to that used on earlier
//  revs of the RFdriver.
//
//  Two different RF gate option are avalible, using a DIO line or a TWI command.
//
//  The control loop runs at 40 Hz vs the 10 Hz used in MIPS. 
//
//  Becasue this interface uses less recources that the older RFdrivers the MIPS system RFdriver2
//  will support 4 module for a total of 8 RF channels.
//
//  MIPS supports SEPROM address of 0x50,0x52,0x54, and 0x56. For these base addresses the 
//  command interface address is 0x70, 0x72, 0x74, and 0x76. This module supports extended
//  address by supporting 0x60,0x62,0x64, and 0x66. For these base addresses the command interface
//  address is 0x78, 0x7A, 0x7C, and 0x7E. Note! you cannot mix these modules using standard 
//  addressing and extended addressing.
//
//  Gordon Anderson
//  Rev 1.0, Initial release
//  Rev 1.1, Jan 4, 2020
//    1.) Update the voltage level reading to make sure it never goes negative.
//  Rev 1.2, June 7, 2021
//    1.) Added piecewise linear calibration support
//    2.) Added extended addressing capability
//  Rev 1.3, August 13, 2023
//    1.) Updated the system to support stand alone operation through the USB port
//        Add the following:
//        - Update all the host commands to use the same syntax as MIPS
//        - Added GNAME and SNAME commands
//        - Added command for rev edit
//
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <avr/dtostrf.h>
#include <Thread.h>
#include <ThreadController.h>

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "RFdriver.h"
#include "Errors.h"
#include "Serial.h"
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>
//#include <SeeedGrayOLED.h>
#include <SoftwareI2C.h>

SoftwareI2C WireS1;

int8_t        TWIadd = 0x50;
const char    Version[] PROGMEM = "RFdriver version 1.3, August 13, 2023";
RFdriverData  rfdriver;
RFDRVstate    sdata[2];
int           recAdd;    
int           Eaddress = 0;
uint32_t      ebuf[128];  // long word aligned 512 byte buffer
uint8_t       *Ebuf = (uint8_t *)ebuf;  // Byte pointer to buffer
RFdriverData  *fptr = (RFdriverData *)Ebuf;
bool          update = true;
bool          ReturnAvalible = false;
uint8_t       Schan = 0;  // Selected channel;
// Calibration variables
float         vcal;         // calibration voltage
int           ccal;         // channel to calibrate
bool          Pcal=false;   // true to to calibrate the positive channel
bool          Ncal=false;   // true to to calibrate the negative channel

uint8_t       PWLch;
uint8_t       PWLph;
uint8_t       PWLn;
int           PWLvalue = -1;

// Gate variables
bool          Gated[2]  = {false,false};
// Gate ISR variables
int           ISRpin[2] = {-1,-1};

SerialBuffer sb;

// Monitored values
ReadBacks    rb[2] = {{-1,-1,-1,-1,-1},{-1,-1,-1,-1,-1}};

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved lost every time
// the sketch is uploaded on the board.
FlashStorage(flash_RFdriverData, RFdriverData);

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

RFdriverData  RFDD_A_Rev_1 = {sizeof(RFdriverData),"RFdriver", 3, 2, 0x20, 0x69, 0x50,
                              1000000, 0.0, 0, RF_MANUAL, 50.0, 20.0, 5, 4, 32, 320, 5, 32, 320, 0, 2383.09, 0, 1, 2621.4, 0,
                              1000000, 0.0, 0, RF_MANUAL, 50.0, 20.0, 7, 6, 32, 320, 7, 32, 320, 2, 2383.09, 0, 3, 2621.4, 0,
                              0,0,
                              -1,-1,
                              SIGNATURE,
                              50,
                              9,6391,8051,9690,11382,12972,14830,16443,18287,20129,0,100,200,300,400,500,600,700,800,900,0,
                              9,6391,8051,9690,11382,12972,14830,16443,18287,20129,0,100,200,300,400,500,600,700,800,900,0,
                              9,6391,8051,9690,11382,12972,14830,16443,18287,20129,0,100,200,300,400,500,600,700,800,900,0,
                              9,6391,8051,9690,11382,12972,14830,16443,18287,20129,0,100,200,300,400,500,600,700,800,900,0,
                             };

// Auto tune parameters
bool TuneRequest   = false;
bool RetuneRequest = false;
bool Tuning        = false;
bool TuneReport    = false;
int  TuneRFChan;
// Tune states
#define TUNE_SCAN_DOWN 1
#define TUNE_SCAN_UP 2

#define MaxNumDown 5

void msTimerIntercept(void);
extern void (*mySysTickHook)(void);
void (*mySysTickHook)(void) = msTimerIntercept;
void msTimerIntercept(void)
{

}

void GateISR(void)
{
  if(ISRpin[0] != -1)
  {
    if(digitalRead(ISRpin[0]) == HIGH)
    {
      // Gate off
      Gated[0] = true;
      UpdateCH1Drive(0);
    }
    else
    {
      // Gate on
      Gated[0] = false;
      UpdateCH1Drive(rfdriver.RFCD[0].DriveLevel);
    }
  }
  if(ISRpin[1] != -1)
  {
    if(digitalRead(ISRpin[1]) == HIGH)
    {
      // Gate off
      Gated[1] = true;
      UpdateCH2Drive(0);      
    }
    else
    {
      // Gate on
      Gated[1] = false;
      UpdateCH2Drive(rfdriver.RFCD[1].DriveLevel);      
    }    
  }  
}

void AddressMatchEvent(void)
{
  recAdd = PERIPH_WIRE.readDataWIRE() >> 1;
}

// This function is called when the master asks for data.
// Send up to 32 bytes from the sb structure
void requestEventProcessor(void)
{
  int num = sb.available();

  if(ReturnAvalible)
  {
    ReturnAvalible = false;
    Wire.write(num & 0x0FF);
    Wire.write((num >> 8) & 0x0FF);
    return;    
  }
  for (int i = 0; i < num; i++)
  {
    if (i >= 30) break;
    Wire.write(sb.read());
  }  
}

// This function is called when the master asks for data.
// Send up to 32 bytes.
void requestEvent(void)
{
  int cmd_add;

  if((TWIadd & 0x20) != 0) cmd_add =  TWIadd | 0x18;
  else cmd_add = TWIadd | 0x20;
 
  Eaddress &= ~0x0100;    // Reset the page address bit
  // Read the actual TWI address to decide what to do
  if(recAdd == (TWIadd+1))
  {
    // True for second page in SEPROM emulation, set the address page bit
    Eaddress |= 0x0100;
  }
  else if(recAdd == cmd_add)
  {
    // True for general commands, process here
    requestEventProcessor();
    return;
  }
  // Always send 32 bytes of data from the SEPROM buffer, send will
  // terminate on NAK if less is wanted.
  // Use the image of the data stored in flash, this image is in the Ebuf
  for(int i=0;i<32;i++) if(Wire.write(Ebuf[Eaddress+i]) == 0) break;
}

// Reads a 16 bit value from the TWI interface, return -1 if two bytes
// were not avalibale
int ReadUnsignedWord(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  if (Wire.available() == 0) return -1;
  i |= Wire.read() << 8;
  return i & 0xFFFF;
}

bool ReadInt(int *i)
{
  if (Wire.available() == 0) return false;
  *i = Wire.read();
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 8;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 16;
  if (Wire.available() == 0) return false;
  *i |= Wire.read() << 24;
  return true;
}

// Reads a 8 bit value from the TWI interface, return -1 if a byte
// was not avalibale
int ReadUnsignedByte(void)
{
  int i;

  if (Wire.available() == 0) return -1;
  i = Wire.read();
  return i & 0xFF;
}

// Reads a 8 bit signed value from the TWI interface, return false if a byte
// was not avalibale or true if ok
bool ReadByte(int8_t *b)
{
  if (Wire.available() == 0) return false;
  *b = Wire.read();
  return true;
}

bool Read16bitInt(int16_t *shortint)
{
  uint8_t *b = (uint8_t *)shortint;

  if (Wire.available() == 0) return false;
  b[0] = Wire.read();
  if (Wire.available() == 0) return false;
  b[1] = Wire.read();
  return true;
}

// Reads a float value from the TWI interface, return false if float
// was not avalibale
bool ReadFloat(float *fval)
{
  int i;
  uint8_t *b;

  b = (uint8_t *)fval;
  for (int j = 0; j < 4; j++)
  {
    if (Wire.available() == 0) return false;
    b[j] = Wire.read();
  }
  return true;
}

void SendByte(byte bval)
{
  sb.write(bval);
}

void SendWord(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  sb.write(b[0]);
  sb.write(b[1]);
}

void SendInt24(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
}

void SendInt(int ival)
{
  uint8_t *b;

  b = (uint8_t *)&ival;
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
  sb.write(b[3]);
}

void SendFloat(float fval)
{
  uint8_t *b;

  b = (uint8_t *)&fval;
  sb.write(b[0]);
  sb.write(b[1]);
  sb.write(b[2]);
  sb.write(b[3]);
}

void receiveEventProcessor(int howMany)
{
  static uint8_t cmd;
  static int i, j, off, count, startI, stopI;
  static int8_t b;
  static int16_t shortint;
  static float fval;

  while (Wire.available() != 0)
  {
    cmd = Wire.read();
    if (serial == &sb)
    {
      if (cmd == ESC) serial = &Serial;
      else PutCh(cmd);
    }
    else switch (cmd)
    {
      case TWI_SERIAL:
        serial = &sb;
        break;
      case TWI_SET_CHAN:
        if((i=ReadUnsignedByte()) == -1) break;
        if((i==1)||(i==2)) Schan = i - 1;
        break;
      case TWI_SET_FREQ:
        if(!ReadInt(&i)) break;
        if(i < 400000) break;
        if(i > 5000000) break;
        rfdriver.RFCD[Schan].Freq = i;
        break;
      case TWI_SET_MODE:
        if((i=ReadUnsignedByte()) == -1) break;
        if(i) rfdriver.RFCD[Schan].RFmode = RF_AUTO;
        else rfdriver.RFCD[Schan].RFmode = RF_MANUAL;
        break;
      case TWI_SET_DRIVE:
        if(!ReadFloat(&fval)) break;
        if(fval > rfdriver.RFCD[Schan].MaxDrive) break;
        if(fval < 0) break;
        rfdriver.RFCD[Schan].DriveLevel = fval;
        break;
      case TWI_SET_VRF:
        if(!ReadFloat(&fval)) break;
        if(fval > 2000) break;
        if(fval < 0) break;
        rfdriver.RFCD[Schan].Setpoint = fval;
        break;
      case TWI_SET_MAXDRV:
        if(!ReadFloat(&fval)) break;
        if(fval > 100) break;
        if(fval < 0) break;
        rfdriver.RFCD[Schan].MaxDrive = fval;
        break;
      case TWI_SET_MAXPWR:
        if(!ReadFloat(&fval)) break;
        if(fval > 100) break;
        if(fval < 0) break;
        rfdriver.RFCD[Schan].MaxPower = fval;
        break;
      case TWI_SET_ATUNE:
        TuneRFChan = Schan;
        TuneRequest = true;
        break;
      case TWI_SET_RTUNE:
        TuneRFChan = Schan;
        RetuneRequest = true;
        break;
      case TWI_SET_CALP:
        if(!ReadFloat(&fval)) break;
        rfdriver.RFCD[Schan].RFpADCchan.m = fval;
        if(!ReadFloat(&fval)) break;
        rfdriver.RFCD[Schan].RFpADCchan.b = fval;
        break;
      case TWI_SET_CALN:
        if(!ReadFloat(&fval)) break;
        rfdriver.RFCD[Schan].RFnADCchan.m = fval;
        if(!ReadFloat(&fval)) break;
        rfdriver.RFCD[Schan].RFnADCchan.b = fval;
        break;
      case TWI_CALP:
        if(!ReadFloat(&fval)) break;
        vcal = fval;
        ccal = Schan;
        Pcal = true;
        break;
      case TWI_CALN:
        if(!ReadFloat(&fval)) break;
        vcal = fval;
        ccal = Schan;
        Ncal = true;
        break;
      case TWI_SET_GATENA:  // Enable the selected DIO line to use for a gate, high gates off
        if((i = ReadUnsignedByte()) == -1) return;
        ISRpin[Schan] = i;
        pinMode(ISRpin[Schan],INPUT);
        attachInterrupt(ISRpin[Schan], GateISR, CHANGE);
        break;
      case TWI_SET_GATEDIS: 
        ISRpin[Schan] = -1;
        break;
      case TWI_SET_GATE:    // If true the RF output is gated off
        if((i = ReadUnsignedByte()) == -1) return;
        if(i)
        {
          Gated[Schan] = true;
          if(Schan == 0) UpdateCH1Drive(0);
          else UpdateCH2Drive(0);
        }
        else
        {
          Gated[Schan] = false;
          if(Schan == 0) UpdateCH1Drive(rfdriver.RFCD[Schan].DriveLevel);
          else UpdateCH2Drive(rfdriver.RFCD[Schan].DriveLevel);          
        }
        break;
      case TWI_SET_PWL:
        if((i=ReadUnsignedByte()) == -1) break;
        PWLch = i & 1;
        if((i=ReadUnsignedByte()) == -1) break;
        PWLph = i & 1;
        if((i=ReadUnsignedByte()) == -1) break;
        PWLn = i;
        if((i=ReadUnsignedWord()) == -1) break;
        PWLvalue = i;
        break;
      case TWI_SET_PWL_N:
        if((i=ReadUnsignedByte()) == -1) break;
        PWLch = i & 1;
        if((i=ReadUnsignedByte()) == -1) break;
        PWLph = i & 1;
        if((i=ReadUnsignedByte()) == -1) break;
        rfdriver.PWLcal[PWLch][PWLph].num = i;
        break;
      case TWI_READ_AVALIBLE:
        // Set flag to return bytes avalible on the next read from TWI
        ReturnAvalible = true;
        break;
      case TWI_READ_DRIVE:
        SendFloat(rfdriver.RFCD[Schan].DriveLevel);
        break;
      case TWI_READ_FREQ:
        SendInt(rfdriver.RFCD[Schan].Freq);
        break;
      case TWI_READ_READBACKS:
        // Copy the readback structure to the output buffer;
        uint8_t  *bptr;
        bptr = (uint8_t *)&rb[Schan];
        for(i=0;i<sizeof(ReadBacks);i++) sb.write(bptr[i]);
        break;
     case TWI_READ_TUNE:
        SendByte(Tuning);
        break;
     case TWI_READ_CALP:
        SendFloat(rfdriver.RFCD[Schan].RFpADCchan.m);
        SendFloat(rfdriver.RFCD[Schan].RFpADCchan.b);
        break;
     case TWI_READ_CALN:
        SendFloat(rfdriver.RFCD[Schan].RFnADCchan.m);
        SendFloat(rfdriver.RFCD[Schan].RFnADCchan.b);
        break;
     case TWI_READ_PWL:
        if((i=ReadUnsignedByte()) == -1) break;
        PWLch = i & 1;
        if((i=ReadUnsignedByte()) == -1) break;
        PWLph = i & 1;
        if((i=ReadUnsignedByte()) == -1) break;
        PWLn = i;
        SendWord(rfdriver.PWLcal[PWLch][PWLph].Value[PWLn]);
        SendWord(rfdriver.PWLcal[PWLch][PWLph].ADCvalue[PWLn]);
     case TWI_READ_PWL_N:
        if((i=ReadUnsignedByte()) == -1) break;
        PWLch = i & 1;
        if((i=ReadUnsignedByte()) == -1) break;
        PWLph = i & 1;
        SendByte(rfdriver.PWLcal[PWLch][PWLph].num);
        break;
     default:
        break;
    }
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  int cmd_add;

  if((TWIadd & 0x20) != 0) cmd_add =  TWIadd | 0x18;
  else cmd_add = TWIadd | 0x20;

  if(howMany == 0) return;
  Eaddress = 0;
  // Read the actual TWI address to decide what to do
  if(recAdd == (TWIadd + 1))
  {
    // True for second page in SEPROM emulation, set the address page bit
    Eaddress |= 0x0100;
  }
  else if(recAdd == cmd_add)
  {
    // True for general commands, process here
    receiveEventProcessor(howMany);
    return;
  }
  // Here to read data into the SEPROM emulation buffer
  // Read the number of bytes received into the buffer, first byte is the address within the selected page
  Eaddress |= Wire.read();
  if(howMany == 1) return;
  // Copy the working data structure to Ebuf for storage into FLASH
  for(int i=0;i<howMany-1;i++) Ebuf[(Eaddress + i) & 0x1FF] = Wire.read();
  // At this point we should update the FLASH assuming the write protection jumper is
  // installed.
  if((Eaddress + howMany) > 1)
  {
    if(digitalRead(TWIADDWRENA) == LOW)
    {
      // Write data to FLASH
      flash_RFdriverData.write(*fptr);
    }
  }
}

// Initalizes the AD5592 as outlined below, all inputs are 0 to 2.5V,
// uses internal reference:
// ADC channel 0, RF channel 1 drive voltage monitor
// ADC channel 1, RF channel 1 drive current monitor
// ADC channel 2, RF channel 2 drive voltage monitor
// ADC channel 3, RF channel 2 drive current monitor
// ADC channel 4, RF channel 1 RF + level output voltage monitor
// ADC channel 5, RF channel 1 RF - level output voltage monitor
// ADC channel 6, RF channel 2 RF + level output voltage monitor
// ADC channel 7, RF channel 2 RF - level output voltage monitor
void RFdriverAD5592init(int8_t addr)
{
   pinMode(addr,OUTPUT);
   digitalWrite(addr,HIGH);
   // General purpose configuration
   AD5592write(addr, 3, 0x0100);
   // Set ext reference
   AD5592write(addr, 11, 0x0200);
   // Set LDAC mode
   AD5592write(addr, 7, 0x0000);
   // Set DO outputs channels
   AD5592write(addr, 8, 0x0000);
   // Set DAC outputs channels
   AD5592write(addr, 5, 0x0000);
   // Set ADC input channels
   AD5592write(addr, 4, 0x00FF);
   // Turn off all pulldowns
   AD5592write(addr, 6, 0x0000);
}

void setup() 
{    
  asm(".global _printf_float");
  asm(".global _scanf_float");
  delay(10);
  // Read the flash config contents into Ebuf and test the signature
  *fptr = flash_RFdriverData.read();
  if(fptr->Signature == SIGNATURE) rfdriver = *fptr;
  else rfdriver = RFDD_A_Rev_1;
  memcpy(Ebuf,(void *)&rfdriver,sizeof(RFdriverData));
  // Set TWI base address
  TWIadd = rfdriver.EEPROMadr;
  if(digitalRead(TWIADD1) == HIGH) TWIadd |= 0x02;
  if(digitalRead(TWIADD2) == HIGH) TWIadd |= 0x04;
  // Init serial communications
  SerialInit();
  // Init SPI
  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(32);
  SPI.setDataMode(SPI_MODE2);
  RFdriverAD5592init(AD5592_CS);
  analogReadResolution(12);
  // Setup TWI as slave to communicate with MIPS.
  Wire.begin(TWIadd);              // join i2c bus
  // Set mask register
  // bits 16 thru 23 are the address mask.
  // This code emulates the SEPROM and uses the base address for the first
  // 256 byte page and the base address + 1 for the second page.
  // Base address + 0x20 for the general command to this FAIMSFB module.
  // For example if base address is 0x50, then 0x51 is page 2 and 0x70 is
  // general commands.
  // This module also supports extended addressing, so if the base address
  // is 0x60 then the command address is 0x78
  PERIPH_WIRE.disableWIRE();
  if((TWIadd & 0x20) == 0) SERCOM3->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_ADDRMASK( 0x21ul );
  else  SERCOM3->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_ADDRMASK( 0x19ul );
  PERIPH_WIRE.enableWIRE();
  // register events
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);
  Wire.onAddressMatch(AddressMatchEvent);
  initPWM();
  // Init the software TWI and init the clock PLL
  WireS1.begin(5, 2);       // sda, scl
    // Init the clock generator and set the frequencies
  SetRef(20000000);
  CY_Init(rfdriver.CLOCKadr);
  SetPLL2freq(rfdriver.CLOCKadr, rfdriver.RFCD[0].Freq);
  SetPLL3freq(rfdriver.CLOCKadr, rfdriver.RFCD[1].Freq);
  // Configure Threads
  SystemThread.setName("Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
  // Print the signon version message
  update = true;
  serial->println(Version);
}

bool UpdateADCvalue(uint8_t SPIcs, ADCchan *achan, float *value, float filter, float (*c2v)(int,ADCchan *))
{
  int   val;
  float fval;

  if((achan->Chan & 0x80) != 0)
  {
    // Here if this is a M0 ADC pin
    val = 0;
    for(int i=0;i<16;i++) val += analogRead(achan->Chan & 0x7F);
    //val = analogRead(achan->Chan & 0x7F) << 4;
    fval = Counts2Value(val,achan);
    if(*value == -1) *value = fval;
    else *value = filter * fval + (1 - filter) * *value;
    return true;    
  }
  if((val = AD5592readADC(SPIcs, achan->Chan)) != -1)
  {
    if(c2v != NULL) fval = c2v(val,achan);
    else fval = Counts2Value(val,achan);
    if(*value == -1) *value = fval;
    else *value = filter * fval + (1 - filter) * *value;
    return true;
  }
  return false;
}

bool UpdateDACvalue(uint8_t SPIcs, DACchan *dchan, float *value, float *svalue)
{
  if((update) || (*value != *svalue))
  {
    AD5592writeDAC(SPIcs, dchan->Chan, Value2Counts(*value,dchan));
    *svalue = *value;
    return true;
  }
  return false;
}

// Auto tune algorithm, procedure is as follows:
// 1.) Set power tp 10%
// 2.) Set frequency to 1MHz
// 3.) Go down in frequency in 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 4.) Go up in frequency from 1MHzin 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 5.) Use the peak found in steps 3 and 4 and sweep in 10K steps using the same procedure in 3 and 4
// 6.) Use the peak found in step 5 and sweep in 1K steps using the same procedure in 3 and 4
// 7.) Done!
//
// Called from the main processing loop, this function does not block, uses a state machine to perform thge logic
// States
//  TUNE_SCAN_DOWN
//  TUNE_SCAN_UP
void RFdriver_tune(void)
{
   static int    TuneStep = 100000, TuneState;
   static float  Max, Current, Last;
   static int    FreqMax, Freq;
   static int    NumDown,Nth;

   if(TuneRequest)
   {
     // Set freq to 1MHz
     rfdriver.RFCD[TuneRFChan].Freq = 1000000;
     // Set drive to 10%
     rfdriver.RFCD[TuneRFChan].DriveLevel = 10;
     Tuning = true;
     TuneStep = 100000;
     Freq = 1000000;
     Last = Max = 0;
     NumDown = -MaxNumDown;
     TuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 80;
     return;
   }
   if(RetuneRequest)
   {
     // Set freq to current
     Freq = rfdriver.RFCD[TuneRFChan].Freq;
     Tuning = true;
     TuneStep = 1000;
     Last = Max = 0;
     NumDown = 0;
     RetuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 80;
     return;
   }
   if(!Tuning) return;
   if(--Nth > 0) return;
   Nth = 80;
   // Here if the system is tuning
   Current = rb[TuneRFChan].RFP + rb[TuneRFChan].RFN;
   switch (TuneState)
   {
     case TUNE_SCAN_DOWN:
        if(Current > Max)
        {
          Max = Current;
          FreqMax = rfdriver.RFCD[TuneRFChan].Freq;
        }
        if(Current <= (Last + 1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == 100000) NumDown = -MaxNumDown;
        }
        rfdriver.RFCD[TuneRFChan].Freq -= TuneStep;
        if((NumDown >= MaxNumDown) || (rfdriver.RFCD[TuneRFChan].Freq < 500000))
        {
          TuneState = TUNE_SCAN_UP;
          rfdriver.RFCD[TuneRFChan].Freq = Freq;
          NumDown = 0;
          if(TuneStep == 100000) NumDown = -MaxNumDown;
        }
        break;
     case TUNE_SCAN_UP:
        if(Current > Max)
        {
          Max = Current;
          FreqMax = rfdriver.RFCD[TuneRFChan].Freq;
        }
        if(Current <= (Last +1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == 100000) NumDown = -MaxNumDown;
        }
        rfdriver.RFCD[TuneRFChan].Freq += TuneStep;
        if((NumDown >= MaxNumDown) || (rfdriver.RFCD[TuneRFChan].Freq > 5000000))
        {
          // Here we have found the peak for this step size, this
          // process repeats until step size is 1KHz
          Freq = FreqMax;
          if(Freq < 500000) Freq = 500000;
          if(Freq > 5000000) Freq = 5000000;
          rfdriver.RFCD[TuneRFChan].Freq = Freq;
          if(TuneStep == 1000)
          {
            // If here we are done!
            Tuning = false;
            return;
          }
          else TuneStep /= 10;
          TuneState = TUNE_SCAN_DOWN;
          NumDown = 0;
        }
        break;
     default:
        break;
   }
   Last = Current;
}

float RFdriverCounts2Volts(int ADCcounts, ADCchan *adcchan)
{
   float Pv;
   int   i;

   if(rfdriver.Rev == 3)
   {
      // Convert to engineering units for the Linear tech level sensors, 5th order correction.
      // y = -62.27195 + 2.06452*x - 0.025363*x^2 + 0.000161919*x^3 - 3.873611e-7*x^4 + 3.255975e-10*x^5
      Pv = Counts2Value(ADCcounts, adcchan);
      Pv = 3.255975e-10 * pow(Pv,5) - 3.873611e-7 * pow(Pv,4) + 0.000161919 * pow(Pv,3) - 0.025363 * pow(Pv,2) + 2.06452 * Pv - 62.27195;
      if(Pv < 0.0) Pv = 0.0;
   }
   else if((rfdriver.Rev == 2) || (rfdriver.Rev == 4))
   {
     // This rev supports linear operation for high voltage, up to 4000Vp-p, this is equations is for the RF level detector circuit.
     // Rev 2 also displays only the RF+ output. Used on the Eiceman project
     Pv = Counts2Value(ADCcounts, adcchan);
     if(Pv < 0.0) Pv = 0.0;
     // This code attempts to correct for nonlinear performance near 0
     float Zc = (float)Counts2Value(0, adcchan) / 0.8;
     if(Pv <= Zc) Pv = Pv - (Zc - Pv) * 4.0;   
   }
   else if(rfdriver.Rev <= 1)
   {
     // This is a linear calibration using the data structure parameters
     Pv = Counts2Value(ADCcounts, adcchan);
     if(Pv < 0.0) Pv = 0.0;
   }  
   else if(rfdriver.Rev == 5)
   {
     // Find the RF channel number and phase, then return the lookup value
     for(int i=0; i<2; i++)
     {
       if(&rfdriver.RFCD[i].RFpADCchan == adcchan) return PWLlookup(i+1,0,ADCcounts);
       if(&rfdriver.RFCD[i].RFnADCchan == adcchan) return PWLlookup(i+1,1,ADCcounts);
     }
   }
   return Pv; 
}

void VRFcontrolLoop(void)
{
  int chan;

  for(chan=0;chan<2;chan++)
  {
     if((rfdriver.RFCD[chan].RFmode == RF_AUTO) && (!Gated[chan]))
     {
        // Calculate error between actual and setpoint
        float error = (rb[chan].RFP + rb[chan].RFN)/2.0 - rfdriver.RFCD[chan].Setpoint;
        if(fabs(error) < 2.0) return;
        rfdriver.RFCD[chan].DriveLevel -= error * 0.005;
        if(rfdriver.RFCD[chan].DriveLevel > rfdriver.RFCD[chan].MaxDrive) rfdriver.RFCD[chan].DriveLevel = rfdriver.RFCD[chan].MaxDrive;
        if(rfdriver.RFCD[chan].DriveLevel < 0) rfdriver.RFCD[chan].DriveLevel = 0;     
     }
  }
}

// This function is called at 40 Hz and it does all the DAC and ADC updates
void Update(void)
{
  int chan;

  for(chan=0;chan<2;chan++)
  {
    if(rfdriver.RFCD[chan].DriveLevel > rfdriver.RFCD[chan].MaxDrive) rfdriver.RFCD[chan].DriveLevel = rfdriver.RFCD[chan].MaxDrive;
    if(rb[chan].PWR > rfdriver.RFCD[chan].MaxPower) rfdriver.RFCD[chan].DriveLevel -= 0.1;
    if(update || (sdata[chan].DriveLevel != rfdriver.RFCD[chan].DriveLevel))
    {
      if(!Gated[chan])
      {
        if(chan == 0) UpdateCH1Drive(rfdriver.RFCD[chan].DriveLevel);
        else UpdateCH2Drive(rfdriver.RFCD[chan].DriveLevel);
        sdata[chan].DriveLevel = rfdriver.RFCD[chan].DriveLevel;
      }
    }
    if(update || (sdata[chan].Freq != rfdriver.RFCD[chan].Freq))
    {
      if(chan == 0) SetPLL2freq(rfdriver.CLOCKadr, rfdriver.RFCD[chan].Freq);
      else SetPLL3freq(rfdriver.CLOCKadr, rfdriver.RFCD[chan].Freq);
      sdata[chan].Freq = rfdriver.RFCD[chan].Freq;
    }
    UpdateADCvalue(AD5592_CS, &rfdriver.RFCD[chan].RFpADCchan, &rb[chan].RFP,FILTER,RFdriverCounts2Volts);
    UpdateADCvalue(AD5592_CS, &rfdriver.RFCD[chan].RFnADCchan, &rb[chan].RFN,FILTER,RFdriverCounts2Volts);
    UpdateADCvalue(AD5592_CS, &rfdriver.RFCD[chan].DriveVADCchan, &rb[chan].V);
    UpdateADCvalue(AD5592_CS, &rfdriver.RFCD[chan].DriveIADCchan, &rb[chan].I);
    rb[chan].PWR = rb[chan].V * rb[chan].I;
  }
  VRFcontrolLoop();
  RFdriver_tune();
  update = false;
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    PutCh(Serial.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

// Adjust the gain for the selected channel to calibrate the readback to match
// the passed parameter, vpp. If vpp is negative then the gain is set to default
// value. Channel number ch is 0 or 1.
void RFcalP(int ch, float vpp)
{
  int brd,ADCraw;
  float MVpp, m;

  if((ch < 0) || (ch > 1)) return;
  if(vpp < 0.0)
  {
    rfdriver.RFCD[ch].RFpADCchan.m = 32;
    return;
  }
  // Read the ADC value for the current settings.
  ADCraw = AD5592readADC(AD5592_CS, rfdriver.RFCD[ch].RFpADCchan.Chan, 100);
  // Save the current gain value
  m = rfdriver.RFCD[ch].RFpADCchan.m;
  // Adj m to find calibration match
  for(int i=0;i<10000;i++)
  {
    MVpp = RFdriverCounts2Volts(ADCraw, &rfdriver.RFCD[ch].RFpADCchan);
    rfdriver.RFCD[ch].RFpADCchan.m -= 10*(vpp - MVpp)/vpp;
    if((vpp - MVpp) < 1) break;
  }
}

void RFcalN(int ch, float vpp)
{
  int brd,ADCraw;
  float MVpp, m;

  if((ch < 0) || (ch > 1)) return;
  if(vpp < 0.0)
  {
    rfdriver.RFCD[ch].RFnADCchan.m = 32;
    return;
  }
  // Read the ADC value for the current settings.
  ADCraw = AD5592readADC(AD5592_CS, rfdriver.RFCD[ch].RFnADCchan.Chan, 100);
  // Save the current gain value
  m = rfdriver.RFCD[ch].RFnADCchan.m;
  // Adj m to find calibration match
  for(int i=0;i<10000;i++)
  {
    MVpp = RFdriverCounts2Volts(ADCraw, &rfdriver.RFCD[ch].RFnADCchan);
    rfdriver.RFCD[ch].RFnADCchan.m -= 10*(vpp - MVpp)/vpp;
    if((vpp - MVpp) < 1) break;
  }
}

void loop() 
{
  ProcessSerial();
  control.run();
  // Process calibration request
  if(Pcal)
  {
    RFcalP(ccal,vcal);
    Pcal = false;
  }
  if(Ncal)
  {
    RFcalN(ccal,vcal);
    Ncal = false;
  }
  // Process gate request

  // Process PWL table read request
  if(PWLvalue != -1)
  {
    if(PWLph == 0) rfdriver.PWLcal[PWLch][0].ADCvalue[PWLn] = AD5592readADC(AD5592_CS, rfdriver.RFCD[PWLch].RFpADCchan.Chan, 100);
    if(PWLph == 1) rfdriver.PWLcal[PWLch][1].ADCvalue[PWLn] = AD5592readADC(AD5592_CS, rfdriver.RFCD[PWLch].RFnADCchan.Chan, 100);
    rfdriver.PWLcal[PWLch][PWLph].Value[PWLn] = PWLvalue;
    PWLvalue = -1;
  }
}

//
// Host command functions
//

void SaveSettings(void)
{
  rfdriver.Signature = SIGNATURE;
  flash_RFdriverData.write(rfdriver);
  SendACK;
}

void RestoreSettings(void)
{
  // Read the flash config contents into Ebuf and test the signature
  *fptr = flash_RFdriverData.read();
  if(fptr->Signature == SIGNATURE) rfdriver = *fptr;
  else
  {
    // copy rfdriver to Ebuf if restore failed
    *fptr = rfdriver;
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;  
}

void Software_Reset(void)
{
  NVIC_SystemReset();  
}

void FormatFLASH(void)
{
  flash_RFdriverData.write(RFDD_A_Rev_1);  
  SendACK;
}

// Tests the channel number if invalid its NAKed and false is returned.
bool IsChannelValid(int channel, bool Response = true)
{
  if ((channel >= 1) && (channel <= 2)) return true;
  if(!Response) return false;
  SetErrorCode(ERR_BADARG);
  SendNAK;
  return false;
}

// Set a channels freqency
void RFfreq(int channel, int freq)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // If freq value is invalid send NAK and exit
  if ((freq < MinFreq) || (freq > MaxFreq)) BADARG;
  // If here ACK the command and set the frequency
  SendACK;
  rfdriver.RFCD[channel-1].Freq = freq;
}

void RFvoltage(char *Chan, char *Val)
{
  int   channel;
  float Voltage;

  sscanf(Chan, "%d", &channel);
  sscanf(Val, "%f", &Voltage);
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // If Drive value is invalid send NAK and exit
  if ((Voltage < 0) || (Voltage > 4000.0)) BADARG;
  // If here ACK the command and set the drive level
  SendACK;
  rfdriver.RFCD[channel-1].Setpoint = Voltage;
}

void RFdrive(char *Chan, char *Val)
{
  int   channel;
  float Drive;

  sscanf(Chan, "%d", &channel);
  sscanf(Val, "%f", &Drive);
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // If Drive value is invalid send NAK and exit
  if ((Drive < 0) || (Drive > rfdriver.RFCD[channel - 1].MaxDrive)) BADARG;
  // If here ACK the command and set the drive level
  SendACK;
  rfdriver.RFCD[channel - 1].DriveLevel = Drive;
}

void RFfreqReport(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Report the channels frequency
  SendACKonly;
  if (!SerialMute) serial->println(rfdriver.RFCD[channel - 1].Freq);
}

void RFvoltageReportP(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  if (!SerialMute) serial->println(rb[channel-1].RFP);
}

void RFvoltageReportN(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  if (!SerialMute) serial->println(rb[channel-1].RFN);
}

void RFdriveReport(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  if (!SerialMute) serial->println(rfdriver.RFCD[channel - 1].DriveLevel);
}

// Reports the voltage setpoint
void RFvoltageReport(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  if (!SerialMute) serial->println(rfdriver.RFCD[channel - 1].Setpoint);
}

void RFheadPower(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  if (!SerialMute) serial->println(rb[channel-1].PWR);
}

void RFmodeReport(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // report the mode
  SendACKonly;
  if(SerialMute) return;
  if(rfdriver.RFCD[channel - 1].RFmode == RF_MANUAL) serial->println("MANUAL");
  else serial->println("AUTO");
}

void RFmodeSet(char *chan, char *mode)
{
  int    channel;
  String sToken;

  sToken = chan;
  channel = sToken.toInt();
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  sToken = mode;
  if((sToken != "AUTO") && (sToken != "MANUAL")) BADARG;
  SendACK;
  if(sToken == "MANUAL") rfdriver.RFCD[channel - 1].RFmode = RF_MANUAL;
  else 
  {
    rfdriver.RFCD[channel - 1].Setpoint = (rb[channel - 1].RFP + rb[channel - 1].RFN) / 2;
    rfdriver.RFCD[channel - 1].RFmode = RF_AUTO;
  }
}

void RFreportAll(void)
{
  int  i;

  if (SerialMute) return;
  SendACKonly;
  for(i=1;i<=2;i++)
  {
    if (!IsChannelValid(i,false)) break;
    if(i > 1) serial->print(",");
    serial->print(rfdriver.RFCD[i - 1].Freq);
    serial->print(",");
    serial->print(rfdriver.RFCD[i - 1].DriveLevel);
    serial->print(",");
    serial->print(rb[i - 1].RFP);
    serial->print(",");
    serial->print(rb[i - 1].RFN);
  }
  serial->println("");
}

void RFautoTune(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Exit if we are already tuning
  if(Tuning) ERR(ERR_TUNEINPROCESS);
  // Exit if not in manual mode for this channel
  if(rfdriver.RFCD[channel - 1].RFmode != RF_MANUAL) ERR(ERR_NOTINMANMODE);
  // Set the tune flag and exit
  SendACK;
  TuneRFChan = channel - 1;
  TuneRequest = true;
  TuneReport = true;   // Causes the auto tune algorithm to send report
}

void RFautoRetune(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  // Exit if we are already tuning
  if(Tuning) ERR(ERR_TUNEINPROCESS);
  // Exit if not in manual mode for this channel
  if(rfdriver.RFCD[channel - 1].RFmode != RF_MANUAL) ERR(ERR_NOTINMANMODE);
  // Set the tune flag and exit
  SendACK;
  TuneRFChan = channel - 1;
  RetuneRequest = true;
  TuneReport = true;   // Causes the auto tune algorithm to send report
}

// Sets the defined channels calibration parameters to the values passed. The 
// parameters are in the ring buffer when this function is called.
// channel,slope, intercept
// This function sets the pos and neg monitor calibration to the same values.
void RFcalParms(void)
{
   char   *Token;
   String sToken;
   int    ch;
   float  m,b;

   while(1)
   {
     // Read all the arguments
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     ch = sToken.toInt();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     m = sToken.toFloat();
     GetToken(true);
     if((Token = GetToken(true)) == NULL) break;
     sToken = Token;
     b = sToken.toFloat();
     if((Token = GetToken(true)) == NULL) break;
     if(Token[0] != '\n') break;
     // Test the channel and exit if error
     if (!IsChannelValid(ch,false)) break;
     rfdriver.RFCD[ch-1].RFpADCchan.m = m;
     rfdriver.RFCD[ch-1].RFpADCchan.b = b;
     rfdriver.RFCD[ch-1].RFnADCchan.m = m;
     rfdriver.RFCD[ch-1].RFnADCchan.b = b;  
     SendACK;
     return;
   }
   // If here then we had bad arguments!
  SetErrorCode(ERR_BADARG);
  SendNAK;
}

// Software calibration functions to make minor adjustment to the RF level readback detectors. 
// These routines interate to converge on the desired output. To use this feature you need
// to set the RF level and read its level then calibrate the channel by defining the desired
// readback level.
void RFcalP(char *channel, char *Vpp)
{
  String sToken;
  int ch;
  float vpp;

  sToken = channel;
  ch = sToken.toInt();
  sToken = Vpp;
  vpp = sToken.toFloat();
  if (!IsChannelValid(ch,false)) return;
  // Signal processing loop to perform the calibration gain adjustment
  vcal = vpp;
  ccal = ch-1;
  Pcal = true;
  SendACK;
}

void RFcalN(char *channel, char *Vpp)
{
  String sToken;
  int ch;
  float vpp;

  sToken = channel;
  ch = sToken.toInt();
  sToken = Vpp;
  vpp = sToken.toFloat();
  if (!IsChannelValid(ch,false)) return;
  // Signal processing loop to perform the calibration gain adjustment
  vcal = vpp;
  ccal = ch-1;
  Ncal = true;
  SendACK;
}

void GetRFpwrLimit(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  if (!SerialMute) serial->println(rfdriver.PowerLimit);
}

void SetRFpwrLimit(int channel, int Power)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  if((Power < 1) || (Power > 100)) BADARG;
  SendACK;
  rfdriver.PowerLimit = Power;
}

void setMaxDrive(int channel, int Drive)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  if((Drive < 1) || (Drive > 100)) BADARG;
  SendACK;
  rfdriver.RFCD[channel-1].MaxDrive = Drive;
}

void getMaxDrive(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  if (!SerialMute) serial->println(rfdriver.RFCD[channel-1].MaxDrive);
}

void setMaxPower(int channel, int Power)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  if((Power < 1) || (Power > 100)) BADARG;
  SendACK;
  rfdriver.RFCD[channel-1].MaxPower = Power;
}

void getMaxPower(int channel)
{
  // If channel is invalid send NAK and exit
  if (!IsChannelValid(channel)) return;
  SendACKonly;
  if (!SerialMute) serial->println(rfdriver.RFCD[channel-1].MaxPower);
}

void ReportRFchan1(void)
{
   serial->println("RF channel 1 values:");
   serial->print(" RF+: "); serial->println(rb[0].RFP); 
   serial->print(" RF-: "); serial->println(rb[0].RFN);  
   serial->print(" V  : "); serial->println(rb[0].V);  
   serial->print(" I  : "); serial->println(rb[0].I);  
   serial->print(" PWR: "); serial->println(rb[0].PWR);    
}

void ReportRFchan2(void)
{
   serial->println("RF channel 1 values:");
   serial->print(" RF+: "); serial->println(rb[1].RFP); 
   serial->print(" RF-: "); serial->println(rb[1].RFN);  
   serial->print(" V  : "); serial->println(rb[1].V);  
   serial->print(" I  : "); serial->println(rb[1].I);  
   serial->print(" PWR: "); serial->println(rb[1].PWR);  
}

void ReportRFlevelADC(int8_t chan)
{
  int val;
  
  if((chan != 1) && (chan != 2))
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;    
  }
  chan--;
  val = AD5592readADC(AD5592_CS, rfdriver.RFCD[chan].RFpADCchan.Chan,100);
  serial->print(val);
  serial->print(",");
  val = AD5592readADC(AD5592_CS, rfdriver.RFCD[chan].RFnADCchan.Chan,100);
  serial->println(val);
}

void Debug(int i)
{
}

void genPWLcalTable(char *channel, char *phase)
{
  String sToken;
  char   *res;
  int    ph,i;
  float  Drive;
  
  sToken = channel;
  PWLch = sToken.toInt();
  if (!IsChannelValid(PWLch,false)) return;
  sToken = phase;
  if((sToken != "RF+") && (sToken != "RF-")) BADARG;
  if(sToken == "RF+") ph = 0;
  else ph = 1;
  // Send the user instructions.
  serial->println("This function will generate a piecewise linear");
  serial->println("calibration table. Set the drive level to reach");
  serial->println("desired calibration points and then enter the");
  serial->println("measured value to the nearest volt. Press enter");
  serial->println("When finished. Voltage must be increasing!");
  // Loop to allow user to adjust drive and enter measured voltage
  rfdriver.PWLcal[PWLch - 1][ph].num = 0;
  i=0;
  Drive = rfdriver.RFCD[PWLch - 1].DriveLevel;
  while(true)
  {
     serial->print("\nPoint ");
     serial->println(i+1);
     res = UserInput("Enter drive level: ", loop);
     sToken = res;
     rfdriver.RFCD[PWLch - 1].DriveLevel = sToken.toFloat();
     if(rfdriver.RFCD[PWLch - 1].DriveLevel > 100) rfdriver.RFCD[PWLch - 1].DriveLevel = 100;
     if(rfdriver.RFCD[PWLch - 1].DriveLevel < 0) rfdriver.RFCD[PWLch - 1].DriveLevel = 0;
     res = UserInput("Enter measured voltage: ", loop);
     if(res == NULL) break;
     sToken = res;
     rfdriver.PWLcal[PWLch - 1][ph].Value[i] = sToken.toInt();
     // Read the ADC raw counts
     if(ph == 0) rfdriver.PWLcal[PWLch - 1][ph].ADCvalue[i] = AD5592readADC(rfdriver.ADCadr, rfdriver.RFCD[PWLch - 1].RFpADCchan.Chan, 100);
     else rfdriver.PWLcal[PWLch - 1][ph].ADCvalue[i] = AD5592readADC(rfdriver.ADCadr, rfdriver.RFCD[PWLch - 1].RFpADCchan.Chan, 100);
     i++;
     rfdriver.PWLcal[PWLch - 1][ph].num = i;
     if(i>=MAXPWL) break;
  }
  serial->println("");
  // Report the table
  serial->print("Number of table entries: ");
  serial->println(rfdriver.PWLcal[PWLch - 1][ph].num);
  for(i=0;i<rfdriver.PWLcal[PWLch - 1][ph].num;i++)
  {
    serial->print(rfdriver.PWLcal[PWLch - 1][ph].Value[i]);
    serial->print(",");
    serial->println(rfdriver.PWLcal[PWLch - 1][ph].ADCvalue[i]);
  }
  // Done!
  serial->println("\nData entry complete!");
  rfdriver.RFCD[PWLch - 1].DriveLevel = Drive;
}

// This function will use the selected piecewise linear table to convert the 
// adcval to output voltage.
// ch = Rf channel 1 through maximum RF channels
// ph = phase, 0 = RF+, 1 = RF-
float PWLlookup(int ch, int ph, int adcval)
{
  int            brd,i;
  PWLcalibration *pwl;
  
  pwl = &rfdriver.PWLcal[ch - 1][ph];
  if(pwl->num < 2) return 0;
  for(i=0;i<pwl->num-1;i++)
  {
    if(adcval < pwl->ADCvalue[i]) break;
    if((adcval >= pwl->ADCvalue[i]) && (adcval <= pwl->ADCvalue[i+1])) break;
  }
  if(i == pwl->num-1) i--;
  // The points at i and i+1 will be used to calculate the output voltage
  // y = y1 + (x-x1) * (y2-y1)/(x2-x1)
  return (float)pwl->Value[i] + ((float)adcval - (float)pwl->ADCvalue[i]) * ((float)pwl->Value[i+1]-(float)pwl->Value[i])/((float)pwl->ADCvalue[i+1] -(float)pwl->ADCvalue[i]);
}

void SetTWIbaseAdd(char *add)
{
  int i;
  
  sscanf(add,"%x",&i);
  rfdriver.EEPROMadr = i;
  SendACK;
}

void GetTWIbaseAdd(void)
{
  SendACKonly;
  serial->println(rfdriver.EEPROMadr,HEX);
}

void getRev(void)
{
  SendACKonly;
  if (!SerialMute) serial->println(rfdriver.Rev);
}

void setRev(int rev)
{
   if((rev < 1) || (rev >10)) BADARG;
   SendACK;
   rfdriver.Rev = rev;
}
