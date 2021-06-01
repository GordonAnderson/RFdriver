#include "Hardware.h"
#include "AtomicBlock.h"

// Counts to value and value to count conversion functions.
// Overloaded for both DACchan and ADCchan structs.
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}

float Counts2Value(int Counts, ADCchan *ad)
{
  return (Counts - ad->b) / ad->m;
}

int Value2Counts(float Value, DACchan *DC)
{
  int counts;

  counts = (Value * DC->m) + DC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

int Value2Counts(float Value, ADCchan *ac)
{
  int counts;

  counts = (Value * ac->m) + ac->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

// AD5592 IO routines. This is a analog and digitial IO chip with
// a SPI interface. The following are low level read and write functions,
// the modules using this device are responsible for initalizing the chip.

// Write to AD5592
void AD5592write(int CS, uint8_t reg, uint16_t val)
{
  int iStat;

  digitalWrite(CS,LOW);
  SPI.transfer(((reg << 3) & 0x78) | (val >> 8));
  SPI.transfer(val & 0xFF);
  digitalWrite(CS,HIGH);
}

// Read from AD5593R
// returns 16 bit value read
int AD5592readWord(int CS)
{
  uint16_t  val;
  
  digitalWrite(CS,LOW);
  val = SPI.transfer16(0);
  digitalWrite(CS,HIGH);
  return val;
}

// Returns -1 on error. Error is flaged if the readback channel does not match the
// requested channel.
// chan is 0 thru 7
int AD5592readADC(int CS, int8_t chan)
{
   uint16_t  val;

   // Write the channel to convert register
   AD5592write(CS, 2, 1 << chan);
   // Dummy read
   digitalWrite(CS,LOW);
   SPI.transfer16(0);
   digitalWrite(CS,HIGH);
   // Read the ADC data 
   digitalWrite(CS,LOW);
   val = SPI.transfer16(0);
   digitalWrite(CS,HIGH);
   // Test the returned channel number
   if(((val >> 12) & 0x7) != chan) return(-1);
   // Left justify the value and return
   val <<= 4;
   return(val & 0xFFF0);
}

int AD5592readADC(int CS, int8_t chan, int8_t num)
{
  int i,j, val = 0;

  for (i = 0; i < num; i++) 
  {
    j = AD5592readADC(CS, chan);
    if(j == -1) return(-1);
    val += j;
  }
  return (val / num);
}

void AD5592writeDAC(int CS, int8_t chan, int val)
{
   uint16_t  d;
   
   // convert 16 bit DAC value into the DAC data data reg format
   d = ((val>>4) & 0x0FFF) | (((uint16_t)chan) << 12) | 0x8000;
   digitalWrite(CS,LOW);
   val = SPI.transfer((uint8_t)(d >> 8));
   val = SPI.transfer((uint8_t)d);
   digitalWrite(CS,HIGH);
}

// End of AD5592 routines

void UpdateCH2Drive(float drive)
{
   if(drive < 0) drive = 0;
   if(drive > 100) drive = 100;
   AtomicBlock< Atomic_RestoreState > a_Block;
   // Disable TCCx
   TCC1->CTRLA.bit.ENABLE = 0;
   while (TCC1->SYNCBUSY.bit.ENABLE);
   // Set duty cycle. value/PER * 100 = % duty cycle
   TCC1->CC[1].reg = (uint16_t)(VARIANT_MCK/DRVPWMFREQ * drive/100.0);
   TCC1->WEXCTRL.bit.OTMX = 1;
   while (TCC1->SYNCBUSY.bit.CC0 || TCC0->SYNCBUSY.bit.CC1);  
   // Enable TCCx
   TCC1->CTRLA.bit.ENABLE = 1;
   while (TCC1->SYNCBUSY.bit.ENABLE);
}

void UpdateCH1Drive(float drive)
{
   if(drive < 0) drive = 0;
   if(drive > 100) drive = 100;
   // Disable TCCx
   AtomicBlock< Atomic_RestoreState > a_Block;
   TCC2->CTRLA.bit.ENABLE = 0;
   while (TCC2->SYNCBUSY.bit.ENABLE);
   // Set duty cycle. value/PER * 100 = % duty cycle
   TCC2->CC[1].reg = (uint16_t)(VARIANT_MCK/DRVPWMFREQ * drive/100.0);
   TCC2->WEXCTRL.bit.OTMX = 0;
   while (TCC2->SYNCBUSY.bit.CC0 || TCC2->SYNCBUSY.bit.CC1);
   // Enable TCCx
   TCC2->CTRLA.bit.ENABLE = 1;
   while (TCC2->SYNCBUSY.bit.ENABLE);  
}


// Set up the PWM channels used for RFdriver drive level
// VARIANT_MCK is clock frequency
//  D3,   TCC0/WO[1], TCC1/WO[3]  ,RF channel 2 drive level
//  PA13, TCC2/WO[1], TCC0/ WO[7] ,RF channel 1 drive level
//
void initPWM(void)
{
// Configure RF channel 2 drive power PWM, set frequency to 50KHz
   pinPeripheral(3, PIO_TIMER_ALT);
   GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCM_TCC0_TCC1);
   while (GCLK->STATUS.bit.SYNCBUSY == 1);
   TCC1->CTRLA.bit.SWRST = 1;
   while (TCC1->SYNCBUSY.bit.SWRST);
   // Disable TCCx
   TCC1->CTRLA.bit.ENABLE = 0;
   while (TCC1->SYNCBUSY.bit.ENABLE);
   // Set prescaler to 1
   TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_GCLK; 
   // Set TCx as normal PWM
   TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
   while ( TCC1->SYNCBUSY.bit.WAVE );
   while (TCC1->SYNCBUSY.bit.CC0 || TCC1->SYNCBUSY.bit.CC1);
   // Set the initial value, determines duty cycle. value/PER * 100 = % duty cycle
   TCC1->CC[1].reg = 0;
   TCC1->WEXCTRL.bit.OTMX = 1;
   while (TCC1->SYNCBUSY.bit.CC0 || TCC1->SYNCBUSY.bit.CC1);
   // Set PER to determine frequency, frequency = VARIANT_MCK / value
   TCC1->PER.reg = VARIANT_MCK/DRVPWMFREQ;
   while (TCC1->SYNCBUSY.bit.PER);
   // Enable TCCx
   TCC1->CTRLA.bit.ENABLE = 1;
   while (TCC1->SYNCBUSY.bit.ENABLE);

// Configure RF channel 1 drive power PWM, set frequency to 50KHz
   pinPeripheral(38, PIO_TIMER);
   GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCM_TCC2_TC3);
   while (GCLK->STATUS.bit.SYNCBUSY == 1);
   TCC2->CTRLA.bit.SWRST = 1;
   while (TCC2->SYNCBUSY.bit.SWRST);
   // Disable TCCx
   TCC2->CTRLA.bit.ENABLE = 0;
   while (TCC2->SYNCBUSY.bit.ENABLE);
   // Set prescaler to 1
   TCC2->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_GCLK; 
   // Set TCx as normal PWM
   TCC2->WAVE.reg = TCC_WAVE_WAVEGEN_NPWM;
   while ( TCC2->SYNCBUSY.bit.WAVE );
   while (TCC2->SYNCBUSY.bit.CC0 || TCC2->SYNCBUSY.bit.CC1);
   // Set the initial value, determines duty cycle. value/PER * 100 = % duty cycle
   TCC2->CC[1].reg = 0;
   TCC2->WEXCTRL.bit.OTMX = 0;
   while (TCC2->SYNCBUSY.bit.CC0 || TCC2->SYNCBUSY.bit.CC1);
   // Set PER to determine frequency, frequency = VARIANT_MCK / value
   TCC2->PER.reg = VARIANT_MCK/DRVPWMFREQ;
   while (TCC2->SYNCBUSY.bit.PER);
   // Enable TCCx
   TCC2->CTRLA.bit.ENABLE = 1;
   while (TCC2->SYNCBUSY.bit.ENABLE);
}

//
// Timer code used to support scan timer interrupt generation.
// Adapted from: https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f
//

void(* callback_func) (void) = NULL;

//this function gets called by the interrupt at <sampleRate>Hertz
void TC5_Handler (void) 
{
  if(callback_func != NULL) callback_func();
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}

//Configures the TC to generate output events at the samplePeriod.
//Configures the TC in Frequency Generation mode, with an event output once
//each period.
 void tcConfigure(int samplePeriod, void(* callback) (void))  // samplePeriod in mS
{
 callback_func = callback;
 // Enable GCLK for TCC2 and TC5 (timer counter input clock)
 GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
 while (GCLK->STATUS.bit.SYNCBUSY);

 tcReset(); //reset TC5

 // Set Timer counter Mode to 16 bits
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
 // Set TC5 mode as match frequency
 TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
 // Determine and set prescaler and enable TC5
 int targetCount = (VARIANT_MCK / 1000) * samplePeriod;
 if((targetCount /= 1) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV2 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV4 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV8 | TC_CTRLA_ENABLE;
 else if((targetCount /= 2) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV64 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_ENABLE;
 else if((targetCount /= 4) <= 65535) TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1024 | TC_CTRLA_ENABLE;
 //set TC5 timer counter
 TC5->COUNT16.CC[0].reg = targetCount; 
 // Configure interrupt request
 NVIC_DisableIRQ(TC5_IRQn);
 NVIC_ClearPendingIRQ(TC5_IRQn);
 NVIC_SetPriority(TC5_IRQn, 0);
 NVIC_EnableIRQ(TC5_IRQn);

 // Enable the TC5 interrupt request
 TC5->COUNT16.INTENSET.bit.MC0 = 1;
 while (tcIsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tcIsSyncing()
{
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tcStartCounter()
{
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tcIsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tcReset()
{
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tcIsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tcDisable()
{
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tcIsSyncing());
}

void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// The function will program the FLASH memory by receiving a file from the USB connected host. 
// The file must be sent in hex and use the following format:
// First the FLASH address in hex and file size, in bytes (decimal) are sent. If the file can
// be burned to FLASH an ACK is sent to the host otherwise a NAK is sent. The process stops
// if a NAK is sent. 
// If an ACK is sent to the host then the host will send the data for the body of the 
// file in hex. After all the data is sent then a 8 bit CRC is sent, in decimal. If the
// crc is correct and ACK is returned.
void ProgramFLASH(char * Faddress,char *Fsize)
{
  static String sToken;
  static uint32_t FlashAddress;
  static int    numBytes,fi,val,tcrc;
  static char   c,buf[3],*Token;
  static byte   fbuf[256],b,crc;
  static byte   vbuf[256];
  static uint32_t start;

  crc = 0;
  FlashAddress = strtol(Faddress, 0, 16);
  sToken = Fsize;
  numBytes = sToken.toInt();
  SendACK;
  fi = 0;
  FlashClass fc((void *)FlashAddress,numBytes);
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;
    sscanf(buf,"%x",&val);
    fbuf[fi++] = val;
    ComputeCRCbyte(&crc,val);
    if(fi == 256)
    {
      fi = 0;
      // Write the block to FLASH
      noInterrupts();
      fc.erase((void *)FlashAddress, 256);
      fc.write((void *)FlashAddress, fbuf, 256);
      // Read back and verify
      fc.read((void *)FlashAddress, vbuf, 256);
      for(int j=0; j<256; j++)
      {
        if(fbuf[j] != vbuf[j])
        {
           interrupts();
           serial->println("FLASH data write error!");
           SendNAK;
           return;   
        }
      }
      interrupts();
      FlashAddress += 256;
      serial->println("Next");
    }
  }
  // If fi is > 0 then write the last partial block to FLASH
  if(fi > 0)
  {
    noInterrupts();
    fc.erase((void *)FlashAddress, fi);
    fc.write((void *)FlashAddress, fbuf, fi);
    // Read back and verify
    fc.read((void *)FlashAddress, vbuf, fi);
    for(int j=0; j<fi; j++)
    {
      if(fbuf[j] != vbuf[j])
      {
         interrupts();
         serial->println("FLASH data write error!");
         SendNAK;
         return;   
      }
    }
    interrupts();
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok exit else delete file and exit
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc)) 
    {
       serial->println("File received from host and written to FLASH.");
       SendACK;
       return;
    }
  }
  serial->println("\nError during file receive from host!");
  SendNAK;
  return;
TimeoutExit:
  serial->println("\nFile receive from host timedout!");
  SendNAK;
  return;
}
