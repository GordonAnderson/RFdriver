#ifndef Hardware_h
#define Hardware_h

// SPI chip selects
#define AD5592_CS     1

// DIO lines
#define TWIADD1       7
#define TWIADD2       6
#define TWIADDWRENA   12

// AD5592 channel assignments, all analog inputs
#define RF1VMON          0           // ADC channel 0, RF channel 1 drive voltage monitor
#define RF1IMON          1           // ADC channel 1, RF channel 1 drive current monitor
#define RF2VMON          2           // ADC channel 2, RF channel 2 drive voltage monitor
#define RF2IMON          3           // ADC channel 3, RF channel 2 drive current monitor
#define RF1LEVP          4           // ADC channel 4, RF channel 1 RF + level output voltage monitor
#define RF1LEVN          5           // ADC channel 5, RF channel 1 RF - level output voltage monitor
#define RF2LEVP          6           // ADC channel 6, RF channel 2 RF + level output voltage monitor
#define RF2LEVN          7           // ADC channel 7, RF channel 2 RF - level output voltage monitor

typedef struct
{
  int8_t  Chan;                   // ADC channel number 0 through max channels for chip.
                                  // If MSB is set then this is a M0 ADC channel number
  float   m;                      // Calibration parameters to convert channel to engineering units
  float   b;                      // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  int8_t  Chan;                   // DAC channel number 0 through max channels for chip
  float   m;                      // Calibration parameters to convert engineering to DAC counts
  float   b;                      // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Function prototypes
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);
void  AD5592write(int CS, uint8_t reg, uint16_t val);
int   AD5592readWord(int CS);
int   AD5592readADC(int CS, int8_t chan);
int   AD5592readADC(int CS, int8_t chan, int8_t num);
void  AD5592writeDAC(int CS, int8_t chan, int val);

void  initPWM(void);
void  UpdateCH1Drive(float drive);
void  UpdateCH2Drive(float drive);

// Firmware field-update over the USB serial command line - see
// ProgramFLASHcmd() in Hardware.cpp for the full protocol and safety design.
//
// The 256KB flash on the SAMD21G18A is partitioned three ways:
//
//   0x00000000  bootloader   8KB   SAM-BA, reserved by this board's linker
//                                  script (variants/feather_m0/linker_scripts/
//                                  gcc/flash_with_bootloader.ld). Never
//                                  touched by anything here - it is the
//                                  recovery path.
//   0x00002000  application  124KB the running firmware. Only ever written by
//                                  the RAM-resident copier at the very end of
//                                  an update.
//   0x00021000  staging      124KB where an incoming update is received and
//                                  verified. Holds no executing code, so it
//                                  is safe to erase and write while the
//                                  firmware runs normally.
//
// The split exists because this chip has a single flash bank: erasing a row
// that holds executing code hard-hangs the CPU (it fetches erased 0xFF as an
// instruction). Streaming an update directly over the running application is
// therefore impossible - that was the Rev 1.4 bug. Staging first means the
// only moment the application is overwritten is inside a RAM-resident routine
// that calls nothing in flash.
//
// FLASH_ROW_SIZE is this chip's NVM erase granularity (4 x 64-byte pages) and
// matches the block size the update protocol already uses.
#define APP_FLASH_START    0x00002000UL
#define APP_FLASH_END      0x00021000UL
#define STAGE_FLASH_START  0x00021000UL
#define STAGE_FLASH_END    0x00040000UL
#define FLASH_ROW_SIZE     256

// Largest image an update can carry: it has to fit in the application
// partition and in staging, which are deliberately the same size (124KB).
#define APP_MAX_SIZE     (APP_FLASH_END - APP_FLASH_START)

void  ProgramFLASHcmd(char *sizeStr);

#endif
