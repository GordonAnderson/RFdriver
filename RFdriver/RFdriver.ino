//
// RFdriver
//
//  This aplication is for the M0 processor on the rev 6.0 RFdriver hardware. The M0 processor 
//  is the same prcessor used on the adafruit feather. The Arduino IDE is used to develope this
//  application. The M0 processor emulate the SEPROM used on the MIPS modules. This RFdriver 
//  interfaces with the MIPS controller through the emulated SEPROM and the TWI interface.
//  
//  The emulated SEPROM memory configuration matches that on the previous hardware versions 
//  of the RFdriver.
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
//  Gordon Anderson
//  Rev 1.0, Initial release
//  Rev 1.1, Jan 4, 2020
//    1.) Update the voltage level reading to make sure it never goes negative.
//
#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
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
const char    Version[] PROGMEM = "RFdriver version 1.1, Jan 4, 2020";
RFdriverData  rfdriver;
RFDRVstate    sdata[2];
int           Eaddress = 0;
int           recAdd;
uint8_t       Ebuf[512];
RFdriverData  *fptr = (RFdriverData *)Ebuf;
bool          update = true;
bool          ReturnAvalible = false;
uint8_t       Schan = 0;  // Selected channel;
// Calibration variables
float         vcal;         // calibration voltage
int           ccal;         // channel to calibrate
bool          Pcal=false;   // true to to calibrate the positive channel
bool          Ncal=false;   // true to to calibrate the negative channel
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
                              SIGNATURE
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
  Eaddress &= ~0x0100;    // Reset the page address bit
  // Read the actual TWI address to decide what to do
  if(recAdd == (TWIadd+1))
  {
    // True for second page in SEPROM emulation, set the address page bit
    Eaddress |= 0x0100;
  }
  else if(recAdd == (TWIadd | 0x20))
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
     default:
        break;
    }
  }
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
void receiveEvent(int howMany)
{
  if(howMany == 0) return;
  Eaddress = 0;
  // Read the actual TWI address to decide what to do
  if(recAdd == (TWIadd + 1))
  {
    // True for second page in SEPROM emulation, set the address page bit
    Eaddress |= 0x0100;
  }
  else if(recAdd == (TWIadd | 0x20))
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
  TWIadd = 0x50;
  if(digitalRead(TWIADD1) == HIGH) TWIadd |= 0x02;
  if(digitalRead(TWIADD2) == HIGH) TWIadd |= 0x04;
  // Read the flash config contents into Ebuf and test the signature
  *fptr = flash_RFdriverData.read();
  if(fptr->Signature == SIGNATURE) rfdriver = *fptr;
  else rfdriver = RFDD_A_Rev_1;
  memcpy(Ebuf,(void *)&rfdriver,sizeof(RFdriverData));
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
  PERIPH_WIRE.disableWIRE();
  SERCOM3->I2CS.ADDR.reg |= SERCOM_I2CS_ADDR_ADDRMASK( 0x21ul );
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
void ProcessSerial(bool scan = true)
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
// value. Channel numbe ch is 0 or 1.
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
    // copy faimsfb to Ebuf if restore failed
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

void Debug(int i)
{
}
