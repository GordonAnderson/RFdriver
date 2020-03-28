#ifndef RFdriver_h
#define RFdriver_h
#include "Hardware.h"

#define FILTER     0.1

#define SIGNATURE  0xAA55A5A5
#define DRVPWMFREQ 50000

#define ESC   27
#define ENQ   5

// TWI commands and constants
#define TWI_SET_CHAN           0x01      // Set the active channel that all commands will reference, byte, channel 1 or 2
#define TWI_SET_FREQ           0x02      // Set frequency, int 32
#define TWI_SET_MODE           0x04      // Set Vrf mode, true = closed loop, bool
#define TWI_SET_DRIVE          0x05      // Drive level, float, percentage
#define TWI_SET_VRF            0x06      // Set the Vrf voltage setpoint, float
#define TWI_SET_MAXDRV         0x07      // Set the maximum drive level, float
#define TWI_SET_MAXPWR         0x08      // Set the maximum power, float
#define TWI_SET_ATUNE          0x09      // Start the auto tune process for selected channel
#define TWI_SET_RTUNE          0x0A      // Start the auto re-tune process for selected channel
#define TWI_SET_CALP           0x0B      // Sets the calibration parameters for VRFP, m and b. two floats
#define TWI_SET_CALN           0x0C      // Sets the calibration parameters for VRFN, m and b. two floats
#define TWI_CALP               0x0D      // The command will adjust the positive channel gain to set the Vpp to the setpoint.
                                         // If the setpoint is negative then the calibration is set to default, float
#define TWI_CALN               0x0E      // The command will adjust the negative channel gain to set the Vpp to the setpoint.
                                         // If the setpoint is negative then the calibration is set to default, float
#define TWI_SET_GATENA         0x0F      // Enable the gate input of the DIO channel passed, when the DIO line is high the RF output 
                                         // is gated off, int
#define TWI_SET_GATEDIS        0x10      // Disable the gate interrupt
#define TWI_SET_GATE           0x11      // Gates the RF off if true, on if false. 
                                         

#define TWI_SERIAL             0x27      // This command enables the TWI port to process serial commands

#define TWI_READ_READBACKS     0x81      // Returns the readback structure
#define TWI_READ_AVALIBLE      0x82      // Returns the number of bytes avalible in output buffer, 16 bit unsigned int
#define TWI_READ_DRIVE         0x83      // Returns the current drive setting, float
#define TWI_READ_FREQ          0x84      // Returns the current frequency setting, int
#define TWI_READ_TUNE          0x85      // Returns the auto tune flag, bool
#define TWI_READ_CALP          0x86      // Returns the calibration parameters for VRFP, m and b. two floats
#define TWI_READ_CALN          0x87      // Returns the calibration parameters for VRFN, m and b. two floats

enum RFdriverMode
{
  RF_MANUAL,
  RF_AUTO,
};

// Current status structure, used to determine if any values have changed. One
// for each channel in module.
typedef struct
{
  int            Freq;                // RF driver frequency in Hz
  float          DriveLevel;          // RF driver level in percentage
  float          Setpoint;            // RF level in volts p-p for automatic control mode
  RFdriverMode   RFmode;              // Defines the RF driver mode of operation
  float          MaxDrive;            // Software limit to drive level
  float          MaxPower;            // Software power limit for RF head
} RFDRVstate;

typedef struct
{
  int            Freq;                // RF driver frequency in Hz
  float          DriveLevel;          // RF driver level in percentage
  float          Setpoint;            // RF level in volts p-p for automatic control mode
  RFdriverMode   RFmode;              // Defines the RF driver mode of operation
  float          MaxDrive;            // Software limit to drive level
  float          MaxPower;            // Software power limit for RF head
  // Hardware specific definitions
  int8_t         PWMchan;             // Define the PWM output channel
  ADCchan        RFpADCchan;          // RF positive phase readback ADC channel
  ADCchan        RFnADCchan;          // RF negative phase readback ADC channel
  ADCchan        DriveVADCchan;       // RF drive voltage monitor ADC channel
  ADCchan        DriveIADCchan;       // RF drive current monitor ADC channel
} RFchannelData;

// One struct for each RF driver board
typedef struct
{
  int16_t        Size;              // This data structures size in bytes
  char           Name[20];          // Holds the board name
  int8_t         Rev;               // Holds the board revision number
  int8_t         NumChannels;       // Number of channels suppoted by this board
  // TWI device addresses
  uint8_t        ADCadr;
  uint8_t        CLOCKadr;
  uint8_t        EEPROMadr;
  RFchannelData  RFCD[2];
  // These parameters really belong in RFchnnelData but there were added after release so placing at end of struct give backwards compatability
  char           RFgateDI[2];       // Gate input, 0 if not used otherwise its the input P-X
  int8_t         RFgateTrig[2];     // Gate level, 0,CHANGE,RISING, or FALLING
  int            Signature;         // Must be 0xAA55A5A5 for valid data
} RFdriverData;


// One struct for each channel, 2 channels in this module
typedef struct
{
  float        RFP;        // RF+ output voltage monitor
  float        RFN;        // RF- output voltage monitor
  float        V;          // Driver supply voltage monitor
  float        I;          // Driver supply current monitor
  float        PWR;        // Driver power level
} ReadBacks;

extern RFdriverData  rfdriver;

// prototypes
bool UpdateADCvalue(uint8_t SPIcs, ADCchan *achan, float *value, float filter = FILTER, float (*c2v)(int,ADCchan *) = Counts2Value);
//bool UpdateADCvalue(uint8_t SPIcs, ADCchan *achan, float *value, float filter = FILTER);
void ReportRFchan1(void);
void ReportRFchan2(void);

#endif
