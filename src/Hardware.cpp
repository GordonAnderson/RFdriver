//
// Hardware.cpp
//
// Low level hardware access for the RFdriver M0 board:
//  - Counts2Value/Value2Counts: linear engineering-unit <-> ADC/DAC count
//    conversion using the m/b calibration pair stored in each ADCchan/DACchan.
//  - AD5592 routines: bit-banged-over-SPI driver for the Analog Devices
//    AD5592R, the analog/digital IO chip used for all of this board's ADC
//    and DAC channels (see RFdriverAD5592init() in RFdriver.cpp for the
//    channel assignment).
//  - initPWM/UpdateCH1Drive/UpdateCH2Drive: direct SAMD21 TCC register setup
//    for the two RF channel drive-level PWM outputs.
//  - tcConfigure/tcStartCounter/tcDisable/tcIsSyncing/tcReset: TC5-based
//    millisecond scan timer support, adapted from
//    https://gist.github.com/nonsintetic/ad13e70f164801325f5f552f84306d6f.
//    Currently unused (the control loop instead runs off the ThreadController
//    in RFdriver.cpp) but kept available should a hardware-timed scan ever be
//    needed.
//  - ComputeCRCbyte/ComputeCRC/ProgramFLASHcmd: host-driven, in-place firmware
//    field update over the USB serial command line (see ProgramFLASHcmd()
//    below for the full protocol and safety design; CRC-8, polynomial 0x1D,
//    matching the same protocol already used by the MIPS/ARB host tooling).
//
#include "Arduino.h"
#include "wiring_private.h"
#include "RFdriver.h"
#include "Hardware.h"
#include "AtomicBlock.h"
#include "Errors.h"
#include "Serial.h"
#include "SPI.h"
#include <FlashStorage.h>

// Forward declarations for functions defined later in this file
bool tcIsSyncing(void);
void tcReset(void);

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
  digitalWrite(CS,LOW);
  SPI.transfer(((reg << 3) & 0x78) | (val >> 8));
  SPI.transfer(val & 0xFF);
  digitalWrite(CS,HIGH);
}

// Reads whatever the AD5592 last had loaded to shift out (e.g. the result of
// a prior conversion request written with AD5592write()); returns the raw
// 16 bit value read
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

// Averages num consecutive single-shot AD5592readADC() reads of the same
// channel; returns -1 if any one of them fails the readback-channel check.
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

// Writes a 16 bit value to the given AD5592 DAC channel.
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

// Sets the RF channel 2 drive-level PWM duty cycle, 0-100%. Channel 2 is
// driven by TCC1 (see initPWM() below for the pin/timer assignment).
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

// Sets the RF channel 1 drive-level PWM duty cycle, 0-100%. Channel 1 is
// driven by TCC2 (see initPWM() below for the pin/timer assignment).
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

// Rough sanity check that row0 (the first FLASH_ROW_SIZE bytes of an incoming
// image) looks like a real Cortex-M0+ vector table for this chip: word 0 is
// the initial stack pointer, which must land somewhere in the SAMD21's 32KB
// SRAM, and word 1 is the Reset_Handler address, which must be a thumb
// address (odd) inside this board's own application flash region. This is
// not a substitute for the CRC check at the end of the transfer - its only
// job is to refuse to touch flash at all for an obviously-wrong file (wrong
// board, truncated download, a text file dropped in by mistake) before a
// single byte has been written.
static bool IsPlausibleVectorTable(const uint8_t *row0)
{
  uint32_t sp, resetHandler;

  memcpy(&sp, row0, 4);
  memcpy(&resetHandler, row0 + 4, 4);
  if((sp < 0x20000000UL) || (sp > 0x20008000UL)) return false;
  if((resetHandler & 1) == 0) return false;
  if((resetHandler < APP_FLASH_START) || (resetHandler >= APP_FLASH_END)) return false;
  return true;
}

// Scratch row buffer for IAP_CopyStagingToAppAndReset() below. It has to live
// in RAM (it does - .bss), be 4-byte aligned for the word accesses, and be a
// file-scope object rather than a local, so the copier never touches the
// stack-heavy paths or anything else that might drag in a flash-resident
// helper.
static uint8_t IAProwBuf[FLASH_ROW_SIZE] __attribute__((aligned(4)));

// Copies the verified image from the staging partition over the application
// partition, row by row, then resets into it. This function never returns.
//
// THIS IS THE ONLY CODE THAT EVER WRITES THE APPLICATION PARTITION, and the
// reason for the whole staging design. This chip has one flash bank and no
// read-while-write: erasing a row makes the entire flash array unreadable
// until the operation completes, and a row that held executing code comes
// back as 0xFF. So from the first erase below until the reset, this function
// must not execute, call, or return into ANY code in flash - not FlashClass,
// not Serial, not memcpy, not even its own caller.
//
// It satisfies that by living in RAM: the section attribute lands it in
// .data, which the standard startup code already copies from flash to RAM
// before main() runs (see Reset_Handler() in the framework's
// cortex_handlers.c), so this needs no linker script changes. Everything it
// touches is either a CPU register, a peripheral register, or IAProwBuf in
// RAM. Note the deliberate ordering inside the loop: the staged row is read
// into RAM *first*, while the NVM is idle and flash is readable, and only
// then is the destination row erased and rewritten from that RAM copy.
//
// Rev 1.4's version of this (IAP_CommitRow0AndReset) was correct as far as it
// went, and was confirmed working on hardware - but it only ever committed
// row 0, because 1.4 wrongly streamed every other row straight over the
// running application. See ProgramFLASHcmd() below.
//
// The build prints "warning: ignoring changed section attributes for .data"
// for this function - expected and harmless (the assembler just means the
// code bytes get .data's existing read/write flags rather than an
// executable one; there's no MPU configured on this board to enforce that
// distinction). Verify after any change to this function that it still lands
// in RAM and calls nothing in flash:
//   arm-none-eabi-nm  .pio/build/adafruit_feather_m0/firmware.elf | grep IAP_
//     -> address must be 0x20xxxxxx, not 0x000xxxxx
//   arm-none-eabi-objdump -d .pio/build/adafruit_feather_m0/firmware.elf
//     -> its disassembly must contain no "bl" to a flash (0x0000xxxx) address
__attribute__((noinline, section(".data")))
static void IAP_CopyStagingToAppAndReset(uint32_t numRows)
{
  volatile uint32_t *src;
  volatile uint32_t *dst;
  uint32_t           appAddr, stageAddr, row, page, w;

  // Nothing may preempt us: every interrupt handler's code is in the
  // application partition we are about to overwrite.
  __disable_irq();

  // Disable automatic page write so each 64-byte page is only committed when
  // we explicitly issue WP below (same sequence FlashStorage's FlashClass
  // uses, just inlined here with no external call).
  NVMCTRL->CTRLB.bit.MANW = 1;

  for (row = 0; row < numRows; row++)
  {
    stageAddr = STAGE_FLASH_START + (row * FLASH_ROW_SIZE);
    appAddr   = APP_FLASH_START   + (row * FLASH_ROW_SIZE);

    // Read the staged row into RAM while the NVM is idle. The pointers are
    // volatile so the compiler cannot turn this loop into a memcpy() call -
    // memcpy lives in flash, and calling it here would be fatal.
    src = (volatile uint32_t *)stageAddr;
    dst = (volatile uint32_t *)IAProwBuf;
    for (w = 0; w < FLASH_ROW_SIZE / 4; w++) dst[w] = src[w];

    // Erase the destination row (NVMCTRL ADDR takes a 16-bit half-word
    // address). From here until the last WP below, flash is unreadable.
    NVMCTRL->ADDR.reg = appAddr / 2;
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
    while (!NVMCTRL->INTFLAG.bit.READY) { }

    // Write it back from the RAM copy, one 64-byte page at a time.
    src = (volatile uint32_t *)IAProwBuf;
    dst = (volatile uint32_t *)appAddr;
    for (page = 0; page < FLASH_ROW_SIZE / 64; page++)
    {
      // PBC: Page Buffer Clear
      NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_PBC;
      while (!NVMCTRL->INTFLAG.bit.READY) { }
      for (w = 0; w < 64 / 4; w++) *dst++ = *src++;
      // WP: Write Page
      NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;
      while (!NVMCTRL->INTFLAG.bit.READY) { }
    }
  }

  // System reset - the exact sequence CMSIS's NVIC_SystemReset() uses,
  // inlined here rather than called, so nothing outside this function is
  // ever invoked from the first erase onward.
  __DSB();
  SCB->AIRCR = (uint32_t)((0x5FAUL << SCB_AIRCR_VECTKEY_Pos) | SCB_AIRCR_SYSRESETREQ_Msk);
  __DSB();
  for (;;) { }
}
// Erases and writes one row into the STAGING partition, then reads it back to
// verify. Returns false on a mismatch.
//
// Unlike Rev 1.4's equivalent, this is safe to call from ordinary
// flash-resident code, because staging never holds executing code. Interrupts
// are still masked around the NVM operation: an erase or write makes the
// WHOLE flash array unreadable while it is in flight, so any interrupt that
// fired would fetch its handler from unreadable flash and fault. That hazard
// belongs to the operation itself, not to which row is being written, and the
// tight poll loop inside FlashClass runs from the NVM cache, which is why this
// works at all.
static bool StageRow(uint32_t addr, byte *data, byte *verifyBuf)
{
  FlashClass fc((void *)addr, FLASH_ROW_SIZE);

  noInterrupts();
  fc.erase();
  fc.write(data);
  fc.read(verifyBuf);
  interrupts();
  return (memcmp(data, verifyBuf, FLASH_ROW_SIZE) == 0);
}

// Re-reads the staged image straight out of flash and recomputes its CRC.
// The per-row read-back in StageRow() already proves each row landed
// correctly; this proves the image as a whole is intact and in the right
// place, and it is the last thing checked before the irreversible copy.
static bool StagedImageCRCok(int numBytes, byte expected)
{
  const uint8_t *p = (const uint8_t *)STAGE_FLASH_START;
  byte           c = 0;

  for(int i=0; i<numBytes; i++) ComputeCRCbyte(&c, p[i]);
  return (c == expected);
}

// Field-updates this module's own firmware, received from the USB-connected
// host as a hex-encoded, CRC-checked binary image - the same protocol the
// MIPS/ARB host tooling already speaks for other modules' FLASH (see
// Comms::ARBupload() in the MIPS host app): the host sends the image size in
// decimal, then the raw bytes of the image as ASCII hex (two characters per
// byte), then a newline, then the 8-bit CRC (poly 0x1D, see ComputeCRC()) of
// the whole image in decimal. "Next" is sent back after each complete row so
// the host can pace itself.
//
// How this works, and why (see also README.md and the Rev 1.5 entry in
// RFdriver.cpp):
//
//  - The incoming image is written into the STAGING partition, never over the
//    running application. This is the whole point of the design. The chip has
//    one flash bank and no read-while-write, so erasing a row that holds
//    executing code hangs the CPU the moment it fetches the erased 0xFF back
//    as an instruction. Rev 1.4 streamed straight over the running
//    application and hard-hung every time it reached a row holding this very
//    function - see the Rev 1.5 entry in RFdriver.cpp for the full postmortem.
//    Staging holds no code, so receiving is completely safe: serial, USB and
//    the command loop keep running normally throughout.
//  - The image always lands at APP_FLASH_START in the end; there is no
//    host-supplied address (that removed a whole class of operator error - a
//    typo'd address used to be able to overwrite anything in flash).
//  - It is rejected before anything is written if it is smaller than one FLASH
//    row, larger than the application partition, or its vector table doesn't
//    look plausible (IsPlausibleVectorTable()).
//  - Every staged row is read back and verified as it is written; then the
//    whole staged image is re-read and CRC'd again (StagedImageCRCok()) before
//    the commit.
//  - Only then does IAP_CopyStagingToAppAndReset() - RAM-resident, calling
//    nothing in flash - copy staging over the application and reset. Any
//    failure before that point (bad CRC, verify mismatch, timeout, implausible
//    image, dropped connection) leaves the application partition completely
//    untouched and the board running normally, with no recovery needed. That
//    is the property Rev 1.4 claimed and did not actually have.
//  - Both RF channels' drive level are forced to 0 before anything else, since
//    the update blocks the normal 25 ms control loop for its entire duration
//    (a bad idea to leave RF output running unsupervised for that long) and
//    the drive PWM hardware otherwise keeps outputting whatever was last
//    commanded, independent of what the CPU is doing.
//
// What this does NOT protect against: power loss or a reset during the final
// copy itself. That window is now a few hundred milliseconds of flash-to-flash
// copying rather than the multi-minute transfer it used to be, but it is still
// there, and a failure inside it leaves the application partition incomplete.
// Recovery is the same physical bootloader path this board already relies on
// for a bad initial programming - open the enclosure, double-tap reset to
// force the board into its SAM-BA bootloader (it appears as a new serial port,
// not a mass-storage drive - this board's bootloader is the classic
// Arduino/bossac one, not the UF2/drag-and-drop kind some other Adafruit SAMD
// boards use), then reflash a known-good build. Note that `pio run -t upload`
// does not work from an already-bootloader state (its 1200-baud touch-reset
// assumes the application is running); use bossac directly, or reflash
// immediately after the double-tap. That fallback is unaffected by anything
// here, since nothing in this file ever touches flash below APP_FLASH_START.
void ProgramFLASHcmd(char *sizeStr)
{
  static String sToken;
  static int    numBytes,fi,val,tcrc;
  static char   c,buf[3],*Token;
  static byte   fbuf[FLASH_ROW_SIZE],crc;
  static byte   vbuf[FLASH_ROW_SIZE];
  static uint32_t start;
  uint32_t      stageAddress,numRows;
  bool          checkedRow0;

  sToken = sizeStr;
  numBytes = sToken.toInt();
  if((numBytes <= FLASH_ROW_SIZE) || (numBytes > (int)APP_MAX_SIZE))
  {
    SetErrorCode(ERR_BADARG);
    SendNAK;
    return;
  }
  // Force RF drive off for the duration - see the safety design note above.
  UpdateCH1Drive(0);
  UpdateCH2Drive(0);
  crc = 0;
  SendACK;
  fi = 0;
  checkedRow0 = false;
  stageAddress = STAGE_FLASH_START;
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
    if(fi == FLASH_ROW_SIZE)
    {
      fi = 0;
      if(!checkedRow0)
      {
        // Row 0 is the vector table. Sanity-check it before we write anything
        // at all, so an obviously-wrong file costs nothing.
        if(!IsPlausibleVectorTable(fbuf))
        {
          serial->println("Image does not look like a valid firmware image for this board - aborted, nothing written.");
          SetErrorCode(ERR_BADARG);
          SendNAK;
          return;
        }
        checkedRow0 = true;
      }
      if(!StageRow(stageAddress, fbuf, vbuf))
      {
        serial->println("FLASH verify error while staging - aborted. The application partition was never touched; the old firmware is still running normally.");
        SendNAK;
        return;
      }
      serial->println("Next");
      stageAddress += FLASH_ROW_SIZE;
    }
  }
  // If fi is > 0 then stage the last partial block, padded with 0xFF (the
  // erased-flash value). No "Next" for a partial row - the host doesn't wait
  // for one (see Comms::ARBupload(), which only waits after full blocks).
  if(fi > 0)
  {
    while(fi < FLASH_ROW_SIZE) fbuf[fi++] = 0xFF;
    if(!StageRow(stageAddress, fbuf, vbuf))
    {
      serial->println("FLASH verify error staging the final block - aborted. The application partition was never touched; the old firmware is still running normally.");
      SendNAK;
      return;
    }
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok commit the staged image and reset; else abort
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc) && checkedRow0)
    {
      // Last check before the point of no return.
      if(!StagedImageCRCok(numBytes, crc))
      {
        serial->println("Staged image failed its read-back CRC - aborted. The application partition was never touched; the old firmware is still running normally.");
        SendNAK;
        return;
      }
      numRows = (numBytes + FLASH_ROW_SIZE - 1) / FLASH_ROW_SIZE;
      serial->println("Image received and verified. Copying it over the application and resetting into the new firmware...");
      delay(50);   // give the message time to actually go out over USB before we reset
      IAP_CopyStagingToAppAndReset(numRows);
      // Never reached.
    }
  }
  serial->println("\nCRC mismatch or malformed transfer - update aborted. The application partition was never touched; the old firmware is still running normally.");
  SendNAK;
  return;
TimeoutExit:
  serial->println("\nFirmware update timed out - aborted. The application partition was never touched; the old firmware is still running normally.");
  SendNAK;
  return;
}
