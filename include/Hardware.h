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
// APP_FLASH_START is the first byte of application flash, right after the
// 8KB UF2 bootloader reserved by this board's linker script
// (variants/feather_m0/linker_scripts/gcc/flash_with_bootloader.ld); code
// below that address is never touched. APP_FLASH_END is one past the last
// byte of flash on the SAMD21G18A. FLASH_ROW_SIZE is this chip's NVM erase
// granularity (4 x 64-byte pages) and matches the block size the update
// protocol already uses.
#define APP_FLASH_START  0x00002000UL
#define APP_FLASH_END    0x00040000UL
#define FLASH_ROW_SIZE   256

void  ProgramFLASHcmd(char *sizeStr);

#endif
